// SPDX-License-Identifier: GPL-2.0-only
/*
 * Loader for Qualcomm Secure Execution Environment applications.
 *
 * The SEE authenticates an application against the metadata in its MDT and
 * takes the application name from there, so all this has to do is reassemble
 * the split image the way the signer laid it out and hand the SEE a physical
 * address.
 */

#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/elf.h>
#include <linux/firmware.h>
#include <linux/firmware/qcom/qcom_scm.h>
#include <linux/firmware/qcom/qcom_tzmem.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/sizes.h>
#include <linux/slab.h>

static char *app_name = "fingerpr";
module_param(app_name, charp, 0444);
MODULE_PARM_DESC(app_name, "Name of the SEE application to load");

static char *fw_path = "qcom/sm8750/xiaomi/piano";
module_param(fw_path, charp, 0444);
MODULE_PARM_DESC(fw_path, "Directory the application image lives in");

static char *region_name = "qseecom-ta";
module_param(region_name, charp, 0444);
MODULE_PARM_DESC(region_name, "Name of the reserved region the SEE keeps applications in");

struct qseecom_ta {
	struct device *dev;
	void *img;
	dma_addr_t img_phys;
	size_t img_size;
	size_t alloc_size;
	u64 bridge;
	phys_addr_t region_phys;
	size_t region_size;
	u32 app_id;
};

static struct qseecom_ta *qseecom_ta;

/*
 * The image is an ELF whose segments were split into one file each. Segment i
 * lives in "<name>.b<ii>" and belongs at the file offset its program header
 * gives, with the MDT - the ELF header, the program headers and the hash
 * segment - at offset zero.
 */
static ssize_t qseecom_ta_image_size(const struct firmware *mdt)
{
	const struct elf64_hdr *ehdr = (const struct elf64_hdr *)mdt->data;
	const struct elf64_phdr *phdrs;
	ssize_t size = mdt->size;
	unsigned int i;

	if (mdt->size < sizeof(*ehdr) || memcmp(ehdr->e_ident, ELFMAG, SELFMAG))
		return -EINVAL;

	if (ehdr->e_ident[EI_CLASS] != ELFCLASS64 ||
	    ehdr->e_phentsize != sizeof(*phdrs))
		return -EINVAL;

	if (ehdr->e_phoff > mdt->size ||
	    array_size(ehdr->e_phnum, sizeof(*phdrs)) > mdt->size - ehdr->e_phoff)
		return -EINVAL;

	phdrs = (const struct elf64_phdr *)(mdt->data + ehdr->e_phoff);

	for (i = 0; i < ehdr->e_phnum; i++) {
		ssize_t end;

		if (check_add_overflow((ssize_t)phdrs[i].p_offset,
				       (ssize_t)phdrs[i].p_filesz, &end))
			return -EINVAL;

		size = max(size, end);
	}

	return size;
}

static int qseecom_ta_assemble(struct qseecom_ta *ta, const struct firmware *mdt)
{
	const struct elf64_hdr *ehdr = (const struct elf64_hdr *)mdt->data;
	const struct elf64_phdr *phdrs;
	unsigned int i;
	int ret;

	phdrs = (const struct elf64_phdr *)(mdt->data + ehdr->e_phoff);
	memcpy(ta->img, mdt->data, mdt->size);

	for (i = 0; i < ehdr->e_phnum; i++) {
		const struct firmware *seg;
		char *name;

		if (!phdrs[i].p_filesz)
			continue;

		name = kasprintf(GFP_KERNEL, "%s/%s.b%02u", fw_path, app_name, i);
		if (!name)
			return -ENOMEM;

		ret = request_firmware(&seg, name, ta->dev);
		if (ret) {
			dev_err(ta->dev, "missing segment %s\n", name);
			kfree(name);
			return ret;
		}

		if (seg->size != phdrs[i].p_filesz) {
			dev_err(ta->dev, "%s is %zu bytes, header says %llu\n",
				name, seg->size, phdrs[i].p_filesz);
			release_firmware(seg);
			kfree(name);
			return -EINVAL;
		}

		memcpy(ta->img + phdrs[i].p_offset, seg->data, seg->size);
		release_firmware(seg);
		kfree(name);
	}

	return 0;
}

static int qseecom_ta_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct firmware *mdt;
	struct qseecom_ta *ta;
	struct reserved_mem *region;
	struct device_node *node;
	ssize_t img_size;
	char *name;
	u32 app_id;
	int ret;

	ret = qcom_scm_qseecom_app_get_id(app_name, &app_id);
	if (!ret) {
		dev_info(dev, "%s is already loaded, app_id %u\n",
			 app_name, app_id);
		return -EEXIST;
	}

	ta = devm_kzalloc(dev, sizeof(*ta), GFP_KERNEL);
	if (!ta)
		return -ENOMEM;

	ta->dev = dev;

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;

	/*
	 * The SEE will not take an application until it knows where it may
	 * keep it, so hand it the region before doing anything else. It takes
	 * the memory over for itself, which is why this has to come from a
	 * no-map reservation rather than from any allocator.
	 */
	node = of_find_node_by_name(NULL, region_name);
	if (!node) {
		dev_err(dev, "no '%s' reserved region\n", region_name);
		return -ENODEV;
	}

	region = of_reserved_mem_lookup(node);
	of_node_put(node);
	if (!region) {
		dev_err(dev, "'%s' is not a reserved region\n", region_name);
		return -ENODEV;
	}

	ta->region_phys = region->base;
	ta->region_size = region->size;

	ret = qcom_scm_qseecom_app_region_notify(ta->region_phys,
						 ta->region_size);
	if (ret) {
		dev_err(dev, "the SEE would not take the %zu byte region at %pa: %d\n",
			ta->region_size, &ta->region_phys, ret);
		return ret;
	}

	dev_info(dev, "the SEE took a %zu byte region at %pa\n",
		 ta->region_size, &ta->region_phys);

	name = kasprintf(GFP_KERNEL, "%s/%s.mdt", fw_path, app_name);
	if (!name)
		return -ENOMEM;

	ret = request_firmware(&mdt, name, dev);
	kfree(name);
	if (ret)
		return dev_err_probe(dev, ret, "cannot read the %s metadata\n",
				     app_name);

	img_size = qseecom_ta_image_size(mdt);
	if (img_size < 0) {
		ret = img_size;
		dev_err(dev, "%s has an unusable metadata segment\n", app_name);
		goto err_release_mdt;
	}

	ta->img_size = img_size;
	ta->alloc_size = PAGE_ALIGN(img_size);
	ta->img = dma_alloc_coherent(dev, ta->alloc_size, &ta->img_phys,
				     GFP_KERNEL);
	if (!ta->img) {
		dev_err(dev, "cannot get %zu contiguous bytes for %s\n",
			ta->alloc_size, app_name);
		ret = -ENOMEM;
		goto err_release_mdt;
	}

	ret = qseecom_ta_assemble(ta, mdt);
	if (ret)
		goto err_free_img;

	/*
	 * The SEE only accepts memory it has been told about, so put the image
	 * behind a shared-memory bridge for the duration of the call.
	 */
	ret = qcom_tzmem_shm_bridge_create(ta->img_phys, ta->alloc_size,
					   &ta->bridge);
	if (ret) {
		dev_err(dev, "cannot bridge the %s image: %d\n", app_name, ret);
		goto err_free_img;
	}

	dev_info(dev, "loading %s: %zu bytes at %pad, %zu of metadata\n",
		 app_name, ta->img_size, &ta->img_phys, mdt->size);

	ret = qcom_scm_qseecom_app_load(ta->img_phys, mdt->size, ta->img_size,
					&ta->app_id);
	qcom_tzmem_shm_bridge_delete(ta->bridge);
	if (ret) {
		dev_err(dev, "the SEE refused %s: %d\n", app_name, ret);
		goto err_free_img;
	}

	dev_info(dev, "%s is running, app_id %u\n", app_name, ta->app_id);

	/* The SEE has its own copy now. */
	dma_free_coherent(dev, ta->alloc_size, ta->img, ta->img_phys);
	ta->img = NULL;
	release_firmware(mdt);

	platform_set_drvdata(pdev, ta);
	qseecom_ta = ta;

	return 0;

err_free_img:
	dma_free_coherent(dev, ta->alloc_size, ta->img, ta->img_phys);
err_release_mdt:
	release_firmware(mdt);
	return ret;
}

static void qseecom_ta_remove(struct platform_device *pdev)
{
	struct qseecom_ta *ta = platform_get_drvdata(pdev);

	qcom_scm_qseecom_app_shutdown(ta->app_id);
	qseecom_ta = NULL;
}

static struct platform_driver qseecom_ta_driver = {
	.probe = qseecom_ta_probe,
	.remove = qseecom_ta_remove,
	.driver = {
		.name = "qcom_qseecom_ta",
	},
};

static struct platform_device *qseecom_ta_pdev;

static int __init qseecom_ta_init(void)
{
	int ret;

	ret = platform_driver_register(&qseecom_ta_driver);
	if (ret)
		return ret;

	qseecom_ta_pdev = platform_device_register_simple("qcom_qseecom_ta",
							  -1, NULL, 0);
	if (IS_ERR(qseecom_ta_pdev)) {
		platform_driver_unregister(&qseecom_ta_driver);
		return PTR_ERR(qseecom_ta_pdev);
	}

	return 0;
}

static void __exit qseecom_ta_exit(void)
{
	platform_device_unregister(qseecom_ta_pdev);
	platform_driver_unregister(&qseecom_ta_driver);
}

module_init(qseecom_ta_init);
module_exit(qseecom_ta_exit);

MODULE_DESCRIPTION("Qualcomm SEE application loader");
MODULE_LICENSE("GPL");
