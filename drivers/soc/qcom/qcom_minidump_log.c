// SPDX-License-Identifier: GPL-2.0-only

#include <linux/init.h>
#include <linux/elf.h>
#include <linux/kernel.h>
#include <linux/printk.h>
#include <linux/slab.h>
#include <linux/soc/qcom/smem.h>

#define SBL_MINIDUMP_SMEM_ID	602
#define MINIDUMP_APSS_ID	0
#define MINIDUMP_REGION_VALID	('V' << 24 | 'A' << 16 | 'L' << 8 | 'I')
#define MINIDUMP_SS_ENCR_NOTREQ	('N' << 8 | 'R')
#define MINIDUMP_SS_ENCR_DONE	('D' << 24 | 'O' << 16 | 'N' << 8 | 'E')
#define MINIDUMP_SS_ENABLED	('E' << 24 | 'N' << 16 | 'B' << 8 | 'L')
#define MINIDUMP_LOG_REGION_COUNT	3
#define MINIDUMP_REGION_COUNT		(MINIDUMP_LOG_REGION_COUNT + 1)
#define MINIDUMP_REGION_NAME_LEN	16
#define MINIDUMP_STRTAB		"\0.shstrtab\0KLOGBUF\0LOG_DESC\0LOG_INFO\0"

static const char * const minidump_log_names[MINIDUMP_LOG_REGION_COUNT] = {
	"KLOGBUF",
	"LOG_DESC",
	"LOG_INFO",
};

struct minidump_elf_header {
	struct elfhdr ehdr;
	struct elf_phdr phdrs[MINIDUMP_LOG_REGION_COUNT];
	struct elf_shdr shdrs[MINIDUMP_LOG_REGION_COUNT + 2];
	char strtab[sizeof(MINIDUMP_STRTAB)];
};

struct minidump_region {
	char name[MINIDUMP_REGION_NAME_LEN];
	__le32 seq_num;
	__le32 valid;
	__le64 address;
	__le64 size;
};

struct minidump_subsystem {
	__le32 status;
	__le32 enabled;
	__le32 encryption_status;
	__le32 encryption_required;
	__le32 region_count;
	__le64 regions_baseptr;
};

struct minidump_global_toc {
	__le32 status;
	__le32 revision;
	__le32 enabled;
	struct minidump_subsystem subsystems[];
};

static struct minidump_region *apss_regions;
static struct minidump_elf_header *minidump_elf;

static void minidump_set_region(struct minidump_region *region,
				const char *name, void *address, size_t size)
{
	strscpy(region->name, name, sizeof(region->name));
	region->valid = cpu_to_le32(MINIDUMP_REGION_VALID);
	region->address = cpu_to_le64(virt_to_phys(address));
	region->size = cpu_to_le64(size);
}

static void minidump_init_elf(const struct printk_ringbuffer_regions *regions)
{
	const void *addresses[MINIDUMP_LOG_REGION_COUNT] = {
		regions->text_data,
		regions->descs,
		regions->infos,
	};
	const size_t sizes[MINIDUMP_LOG_REGION_COUNT] = {
		regions->text_data_size,
		regions->descs_size,
		regions->infos_size,
	};
	struct elf_shdr *strtab = &minidump_elf->shdrs[1];
	size_t data_offset = sizeof(*minidump_elf);
	size_t name_offset = sizeof("\0.shstrtab");
	int i;

	memcpy(minidump_elf->ehdr.e_ident, ELFMAG, SELFMAG);
	minidump_elf->ehdr.e_ident[EI_CLASS] = ELF_CLASS;
	minidump_elf->ehdr.e_ident[EI_DATA] = ELF_DATA;
	minidump_elf->ehdr.e_ident[EI_VERSION] = EV_CURRENT;
	minidump_elf->ehdr.e_type = ET_CORE;
	minidump_elf->ehdr.e_machine = ELF_ARCH;
	minidump_elf->ehdr.e_version = EV_CURRENT;
	minidump_elf->ehdr.e_ehsize = sizeof(minidump_elf->ehdr);
	minidump_elf->ehdr.e_phoff = offsetof(struct minidump_elf_header, phdrs);
	minidump_elf->ehdr.e_phentsize = sizeof(minidump_elf->phdrs[0]);
	minidump_elf->ehdr.e_phnum = MINIDUMP_LOG_REGION_COUNT;
	minidump_elf->ehdr.e_shoff = offsetof(struct minidump_elf_header, shdrs);
	minidump_elf->ehdr.e_shentsize = sizeof(minidump_elf->shdrs[0]);
	minidump_elf->ehdr.e_shnum = ARRAY_SIZE(minidump_elf->shdrs);
	minidump_elf->ehdr.e_shstrndx = 1;

	memcpy(minidump_elf->strtab, MINIDUMP_STRTAB,
	       sizeof(minidump_elf->strtab));
	strtab->sh_name = 1;
	strtab->sh_type = SHT_STRTAB;
	strtab->sh_offset = offsetof(struct minidump_elf_header, strtab);
	strtab->sh_size = sizeof(minidump_elf->strtab);

	for (i = 0; i < MINIDUMP_LOG_REGION_COUNT; i++) {
		struct elf_phdr *phdr = &minidump_elf->phdrs[i];
		struct elf_shdr *shdr = &minidump_elf->shdrs[i + 2];

		phdr->p_type = PT_LOAD;
		phdr->p_offset = data_offset;
		phdr->p_vaddr = (elf_addr_t)addresses[i];
		phdr->p_paddr = virt_to_phys(addresses[i]);
		phdr->p_filesz = sizes[i];
		phdr->p_memsz = sizes[i];
		phdr->p_flags = PF_R | PF_W;

		shdr->sh_name = name_offset;
		shdr->sh_type = SHT_PROGBITS;
		shdr->sh_flags = SHF_WRITE;
		shdr->sh_addr = (elf_addr_t)addresses[i];
		shdr->sh_offset = data_offset;
		shdr->sh_size = sizes[i];

		data_offset += sizes[i];
		name_offset += strlen(minidump_log_names[i]) + 1;
	}
}

static int __init qcom_minidump_log_init(void)
{
	struct printk_ringbuffer_regions log_regions;
	struct minidump_global_toc *global_toc;
	struct minidump_subsystem *apss_toc;
	phys_addr_t regions_phys;
	size_t toc_size;

	global_toc = qcom_smem_get(QCOM_SMEM_HOST_ANY,
				   SBL_MINIDUMP_SMEM_ID, &toc_size);
	if (IS_ERR(global_toc))
		return PTR_ERR(global_toc);

	if (toc_size < struct_size(global_toc, subsystems, 1) ||
	    !le32_to_cpu(global_toc->status))
		return -ENODEV;

	apss_regions = kcalloc(MINIDUMP_REGION_COUNT,
				 sizeof(*apss_regions), GFP_KERNEL);
	if (!apss_regions)
		return -ENOMEM;
	minidump_elf = kzalloc(sizeof(*minidump_elf), GFP_KERNEL);
	if (!minidump_elf) {
		kfree(apss_regions);
		apss_regions = NULL;
		return -ENOMEM;
	}

	log_buf_regions_get(&log_regions);
	minidump_init_elf(&log_regions);
	minidump_set_region(&apss_regions[0], "KELF_HDR",
			    minidump_elf, sizeof(*minidump_elf));
	minidump_set_region(&apss_regions[1], "KLOGBUF",
			    log_regions.text_data, log_regions.text_data_size);
	minidump_set_region(&apss_regions[2], "LOG_DESC",
			    log_regions.descs, log_regions.descs_size);
	minidump_set_region(&apss_regions[3], "LOG_INFO",
			    log_regions.infos, log_regions.infos_size);

	regions_phys = virt_to_phys(apss_regions);
	apss_toc = &global_toc->subsystems[MINIDUMP_APSS_ID];
	apss_toc->enabled = 0;
	apss_toc->status = 0;
	smp_wmb();

	apss_toc->encryption_status = cpu_to_le32(MINIDUMP_SS_ENCR_DONE);
	apss_toc->encryption_required = cpu_to_le32(MINIDUMP_SS_ENCR_NOTREQ);
	apss_toc->regions_baseptr = cpu_to_le64(regions_phys);
	apss_toc->region_count = cpu_to_le32(MINIDUMP_REGION_COUNT);

	/* Publish the initialized region table before enabling the subsystem. */
	smp_wmb();
	apss_toc->status = cpu_to_le32(1);
	apss_toc->enabled = cpu_to_le32(MINIDUMP_SS_ENABLED);

	pr_info("registered KLOGBUF=%pa LOG_DESC=%pa LOG_INFO=%pa\n",
		&(phys_addr_t) { virt_to_phys(log_regions.text_data) },
		&(phys_addr_t) { virt_to_phys(log_regions.descs) },
		&(phys_addr_t) { virt_to_phys(log_regions.infos) });

	return 0;
}
late_initcall(qcom_minidump_log_init);