// SPDX-License-Identifier: GPL-2.0-only
//
// FourSemi FS19xx smart speaker amplifier
//
// Copyright (c) 2026 Junhao Xie <bigfoot@radxa.com>
//
// The FS19xx family is a boosted class-D amplifier with an I2S input and an
// I2S feedback output carrying the measured speaker current and voltage.
// This driver covers the plain amplifier: the part is used here in the
// vendor's "non-DSP" mode, where the on-chip speaker protection runs from
// the factory trim and no coefficient blob has to be downloaded.

#include <linux/bitfield.h>
#include <linux/crc16.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/regmap.h>

#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/tlv.h>

#define FS19XX_STATUS			0x00
#define FS19XX_DEVID			0x03
#define FS19XX_REVID			0x04
#define FS19XX_CHIPINI			0x0e
#define FS19XX_OTPACC			0x0b
#define FS19XX_PWRCTRL			0x10
#define FS19XX_PWRCTRL_PWDN		BIT(0)
#define FS19XX_PWRCTRL_I2CR		BIT(1)
#define FS19XX_SYSCTRL			0x11
#define FS19XX_SPKCOEF			0x14
#define FS19XX_AUDIOCTRL		0x16
#define FS19XX_AUDIOCTRL_VOL		GENMASK(15, 8)
#define FS19XX_I2SCTRL			0x17
#define FS19XX_I2SCTRL_SR		GENMASK(15, 12)
#define FS19XX_I2SCTRL_DOE		BIT(11)
#define FS19XX_I2SCTRL_CHS12		GENMASK(4, 3)
#define FS19XX_TDMCTRL			0x19
#define FS19XX_TDMCTRL_TDMEN		BIT(15)
#define FS19XX_DACCTRL			0x30
#define FS19XX_TSCTRL			0x4c
#define FS19XX_TSCTRL_TSEN		BIT(3)
#define FS19XX_PLLCTRL1			0xa1
#define FS19XX_PLLCTRL2			0xa2
#define FS19XX_PLLCTRL3			0xa3

/* Unlock key for the OTP-shadowed registers, of which the PLL is one. */
#define FS19XX_OTP_ACC_KEY2		0xca91

/* CHIPINI reports 0x0003 once the power-on register load has finished. */
#define FS19XX_CHIPINI_DONE		0x0003

#define FS19XX_DACCTRL_RUN		0x0210
#define FS19XX_DACCTRL_STOP		0x0310

/* OSCEN | VBGEN | ZMEN | ZDPEN | PLLEN | BSTEN | AMPEN */
#define FS19XX_SYSCTRL_ON		0x00ef

#define FS19XX_ID_FS1958		0x17
#define FS19XX_ID_FS19XX		0x29

#define FS19XX_RESET_TRIES		5

/*
 * The vendor ships the per-speaker tuning as a ".fsm" container: a file
 * header, then one descriptor table per amplifier on the board, keyed by
 * I2C address. Each table carries a list of register writes and a little
 * speaker information block. Everything is little endian and unaligned.
 */
#define FS19XX_FW_NAME			"fs19xx.fsm"

#define FS19XX_DSC_DEV_INFO		0
#define FS19XX_DSC_SPK_INFO		1
#define FS19XX_DSC_REG_COMMON		2
#define FS19XX_DSC_REG_SCENES		3

/* Register tables are tagged with the scenes they belong to. */
#define FS19XX_SCENE_MUSIC		BIT(0)

/* Index of the temperature coefficient in the speaker information block. */
#define FS19XX_SPK_INFO_TEMPR_COEF	1

/* A pseudo register position, used to encode delays rather than a write. */
#define FS19XX_FW_REG_PSEUDO		0xf
#define FS19XX_FW_PSEUDO_DELAY		1

struct fs19xx_fw_index {
	__le16 offset;
	__le16 type;
} __packed;

struct fs19xx_fw_header {
	__le16 version;
	char customer[8];
	char project[8];
	__le32 date;
	__le16 size;
	__le16 crc16;
	__le16 ndev;
	struct fs19xx_fw_index index[];
} __packed;

struct fs19xx_fw_dev {
	__le16 preset_ver;
	char project[8];
	char customer[8];
	__le32 date;
	__le16 data_len;
	__le16 crc16;
	__le16 len;
	__le16 bus;
	__le16 addr;
	__le16 dev_type;
	__le16 npreset;
	__le16 reg_scenes;
	__le16 eq_scenes;
	struct fs19xx_fw_index index[];
} __packed;

struct fs19xx_fw_reg {
	__le16 ctl;			/* addr:8, pos:4, len:4 */
	__le16 value;
} __packed;

struct fs19xx_fw_scene_reg {
	__le16 scene;
	struct fs19xx_fw_reg reg;
} __packed;

#define FS19XX_CHS12_LEFT		1
#define FS19XX_CHS12_RIGHT		2
#define FS19XX_CHS12_MONO		3

struct fs19xx_srate {
	unsigned int rate;
	unsigned int val;
};

static const struct fs19xx_srate fs19xx_srates[] = {
	{   8000, 0x1 },
	{  16000, 0x3 },
	{  32000, 0x7 },
	{  44100, 0x8 },
	{  48000, 0x9 },
	{  88200, 0xa },
	{  96000, 0xb },
	{ 176400, 0xc },
	{ 192000, 0xd },
};

struct fs19xx_pll {
	unsigned int bclk;
	u16 c1, c2, c3;
};

static const struct fs19xx_pll fs19xx_plls[] = {
	{   256000, 0x0260, 0x0540, 0x0001 },
	{   512000, 0x0260, 0x0540, 0x0002 },
	{  1024000, 0x0260, 0x0540, 0x0004 },
	{  1024032, 0x0160, 0x0380, 0x0004 },
	{  1411200, 0x0260, 0x0460, 0x0005 },
	{  1536000, 0x0260, 0x0540, 0x0006 },
	{  2048032, 0x0160, 0x0380, 0x0008 },
	{  2822400, 0x0260, 0x0460, 0x000a },
	{  3072000, 0x0260, 0x0540, 0x000c },
	{  6144000, 0x0260, 0x0540, 0x0018 },
	{ 12288000, 0x0260, 0x02a0, 0x0018 },
};

struct fs19xx_priv {
	struct regmap *regmap;
	struct device *dev;
	struct gpio_desc *reset;
	unsigned int devid;
	unsigned int revid;
	unsigned int chs12;
};

/*
 * The part has no reset input that clears the register file on its own: the
 * shutdown pin only gates the output stage. Registers are returned to their
 * power-on values through the I2C reset bit, and CHIPINI then reports when
 * the trim reload has finished.
 */
static int fs19xx_soft_reset(struct fs19xx_priv *fs19xx)
{
	unsigned int val;
	int ret, i;

	for (i = 0; i < FS19XX_RESET_TRIES; i++) {
		regmap_write(fs19xx->regmap, FS19XX_PWRCTRL,
			     FS19XX_PWRCTRL_I2CR);
		/* The reset is not acknowledged; flush it with a read. */
		regmap_read(fs19xx->regmap, FS19XX_PWRCTRL, &val);
		fsleep(15000);

		ret = regmap_write(fs19xx->regmap, FS19XX_PWRCTRL,
				   FS19XX_PWRCTRL_PWDN);
		if (ret)
			continue;

		ret = regmap_read(fs19xx->regmap, FS19XX_CHIPINI, &val);
		if (!ret && val == FS19XX_CHIPINI_DONE)
			return 0;
	}

	dev_err(fs19xx->dev, "device did not come out of reset\n");
	return -ETIMEDOUT;
}

/*
 * A table entry either writes a whole register, updates a field of one, or
 * is a pseudo entry asking for a delay. The remaining pseudo entries poll a
 * bit and are only used by paths this driver does not take.
 */
static int fs19xx_fw_write_reg(struct fs19xx_priv *fs19xx,
			       const struct fs19xx_fw_reg *reg)
{
	u16 ctl = le16_to_cpu(reg->ctl);
	u16 value = le16_to_cpu(reg->value);
	u8 addr = ctl & 0xff;
	u8 pos = (ctl >> 8) & 0xf;
	u8 len = (ctl >> 12) & 0xf;

	if (pos == FS19XX_FW_REG_PSEUDO && len) {
		if (len == FS19XX_FW_PSEUDO_DELAY)
			fsleep(value * USEC_PER_MSEC);
		return 0;
	}

	if (!pos && len == 15)
		return regmap_write(fs19xx->regmap, addr, value);

	return regmap_update_bits(fs19xx->regmap, addr,
				  GENMASK(pos + len, pos), value << pos);
}

/* Every offset in the container is untrusted, so bound each one. */
static const void *fs19xx_fw_at(const struct firmware *fw, size_t offset,
				size_t need)
{
	if (offset > fw->size || need > fw->size - offset)
		return NULL;

	return fw->data + offset;
}

/*
 * A table is a count followed by that many fixed-size entries. Return the
 * first entry and the count, or NULL if the table runs past the file.
 */
static const void *fs19xx_fw_table(const struct firmware *fw, size_t offset,
				   size_t entry_size, unsigned int *count)
{
	const __le16 *len = fs19xx_fw_at(fw, offset, sizeof(*len));

	if (!len)
		return NULL;

	*count = le16_to_cpup(len);

	return fs19xx_fw_at(fw, offset + sizeof(*len), *count * entry_size);
}

static int fs19xx_fw_apply_dev(struct fs19xx_priv *fs19xx,
			       const struct firmware *fw, size_t dev_offset)
{
	/*
	 * The speaker coefficient has to be in place before the register
	 * tables run, and the scene table refines what the common table
	 * sets, so walk the descriptors in this order rather than in the
	 * order the container happens to list them.
	 */
	static const u16 order[] = {
		FS19XX_DSC_SPK_INFO,
		FS19XX_DSC_REG_COMMON,
		FS19XX_DSC_REG_SCENES,
	};
	const struct fs19xx_fw_dev *dev = (const void *)(fw->data + dev_offset);
	size_t index_offset = dev_offset + sizeof(*dev);
	unsigned int entries = le16_to_cpu(dev->len);
	const struct fs19xx_fw_index *index;
	unsigned int i, j, k, count;
	u16 coef;
	int ret;

	index = fs19xx_fw_at(fw, index_offset, entries * sizeof(*index));
	if (!index)
		return -EINVAL;

	for (k = 0; k < ARRAY_SIZE(order); k++) {
		for (i = 0; i < entries; i++) {
			size_t offset;

			if (le16_to_cpu(index[i].type) != order[k])
				continue;

			/*
			 * Offsets within a device are relative to its own
			 * index table, not to the start of the file.
			 */
			offset = index_offset + le16_to_cpu(index[i].offset);

			switch (order[k]) {
			case FS19XX_DSC_SPK_INFO: {
				const __le16 *info;

				info = fs19xx_fw_table(fw, offset, sizeof(*info),
						       &count);
				if (!info)
					return -EINVAL;
				if (count <= FS19XX_SPK_INFO_TEMPR_COEF)
					break;

				coef = le16_to_cpu(info[FS19XX_SPK_INFO_TEMPR_COEF]);
				ret = regmap_write(fs19xx->regmap, FS19XX_SPKCOEF,
						   coef << 1);
				if (ret)
					return ret;
				break;
			}
			case FS19XX_DSC_REG_COMMON: {
				const struct fs19xx_fw_reg *reg;

				reg = fs19xx_fw_table(fw, offset, sizeof(*reg),
						      &count);
				if (!reg)
					return -EINVAL;

				for (j = 0; j < count; j++) {
					ret = fs19xx_fw_write_reg(fs19xx, &reg[j]);
					if (ret)
						return ret;
				}
				dev_dbg(fs19xx->dev, "applied %u common registers\n",
					count);
				break;
			}
			case FS19XX_DSC_REG_SCENES: {
				const struct fs19xx_fw_scene_reg *reg;
				unsigned int applied = 0;

				reg = fs19xx_fw_table(fw, offset, sizeof(*reg),
						      &count);
				if (!reg)
					return -EINVAL;

				for (j = 0; j < count; j++) {
					if (!(le16_to_cpu(reg[j].scene) &
					      FS19XX_SCENE_MUSIC))
						continue;

					ret = fs19xx_fw_write_reg(fs19xx,
								  &reg[j].reg);
					if (ret)
						return ret;
					applied++;
				}
				dev_dbg(fs19xx->dev, "applied %u scene registers\n",
					applied);
				break;
			}
			}
		}
	}

	return 0;
}

/*
 * Without the container the part still plays: it falls back on the trim
 * programmed at the factory. What the container adds is the tuning for the
 * speaker this amplifier actually drives, so a missing file is worth a note
 * but not a probe failure.
 */
static int fs19xx_load_firmware(struct fs19xx_priv *fs19xx, u16 addr)
{
	const struct fs19xx_fw_header *hdr;
	const struct fs19xx_fw_index *index;
	const struct firmware *fw;
	unsigned int i, ndev;
	size_t crc_offset;
	u16 size;
	int ret;

	ret = firmware_request_nowarn(&fw, FS19XX_FW_NAME, fs19xx->dev);
	if (ret) {
		dev_dbg(fs19xx->dev, "no %s, using the factory trim\n",
			FS19XX_FW_NAME);
		return 0;
	}

	hdr = fs19xx_fw_at(fw, 0, sizeof(*hdr));
	if (!hdr) {
		ret = -EINVAL;
		goto out;
	}

	size = le16_to_cpu(hdr->size);
	if (size != fw->size) {
		dev_err(fs19xx->dev, "%s: size %u does not match the file (%zu)\n",
			FS19XX_FW_NAME, size, fw->size);
		ret = -EINVAL;
		goto out;
	}

	/* The checksum covers everything from the device count onwards. */
	crc_offset = offsetof(struct fs19xx_fw_header, ndev);
	if (crc16(0, fw->data + crc_offset, size - crc_offset) !=
	    le16_to_cpu(hdr->crc16)) {
		dev_err(fs19xx->dev, "%s: bad checksum\n", FS19XX_FW_NAME);
		ret = -EINVAL;
		goto out;
	}

	ndev = le16_to_cpu(hdr->ndev);
	index = fs19xx_fw_at(fw, sizeof(*hdr), ndev * sizeof(*index));
	if (!index) {
		ret = -EINVAL;
		goto out;
	}

	dev_dbg(fs19xx->dev, "%s: %.8s %.8s, %u devices\n", FS19XX_FW_NAME,
		hdr->customer, hdr->project, ndev);

	for (i = 0; i < ndev; i++) {
		const struct fs19xx_fw_dev *dev;
		size_t offset = le16_to_cpu(index[i].offset);

		if (le16_to_cpu(index[i].type) != FS19XX_DSC_DEV_INFO)
			continue;

		dev = fs19xx_fw_at(fw, offset, sizeof(*dev));
		if (!dev) {
			ret = -EINVAL;
			goto out;
		}

		if (le16_to_cpu(dev->addr) != addr)
			continue;

		ret = fs19xx_fw_apply_dev(fs19xx, fw, offset);
		if (ret)
			dev_err(fs19xx->dev, "%s: failed to apply the tuning: %d\n",
				FS19XX_FW_NAME, ret);
		goto out;
	}

	dev_warn(fs19xx->dev, "%s: no tuning for address %#04x\n",
		 FS19XX_FW_NAME, addr);
	ret = 0;
out:
	release_firmware(fw);

	return ret;
}

static int fs19xx_set_pll(struct fs19xx_priv *fs19xx, unsigned int bclk)
{
	int i, ret;

	for (i = 0; i < ARRAY_SIZE(fs19xx_plls); i++)
		if (fs19xx_plls[i].bclk == bclk)
			break;

	if (i == ARRAY_SIZE(fs19xx_plls)) {
		dev_err(fs19xx->dev, "unsupported bit clock %u\n", bclk);
		return -EINVAL;
	}

	ret = regmap_write(fs19xx->regmap, FS19XX_OTPACC, FS19XX_OTP_ACC_KEY2);
	if (ret)
		return ret;

	ret = regmap_write(fs19xx->regmap, FS19XX_PLLCTRL1, fs19xx_plls[i].c1);
	ret = ret ?: regmap_write(fs19xx->regmap, FS19XX_PLLCTRL2,
				  fs19xx_plls[i].c2);
	ret = ret ?: regmap_write(fs19xx->regmap, FS19XX_PLLCTRL3,
				  fs19xx_plls[i].c3);

	return regmap_write(fs19xx->regmap, FS19XX_OTPACC, 0) ?: ret;
}

static int fs19xx_hw_params(struct snd_pcm_substream *substream,
			    struct snd_pcm_hw_params *params,
			    struct snd_soc_dai *dai)
{
	struct fs19xx_priv *fs19xx = snd_soc_component_get_drvdata(dai->component);
	unsigned int rate = params_rate(params);
	unsigned int bclk;
	unsigned int sr;
	int i, ret;

	for (i = 0; i < ARRAY_SIZE(fs19xx_srates); i++)
		if (fs19xx_srates[i].rate == rate)
			break;

	if (i == ARRAY_SIZE(fs19xx_srates)) {
		dev_err(fs19xx->dev, "unsupported rate %u\n", rate);
		return -EINVAL;
	}
	sr = fs19xx_srates[i].val;

	/*
	 * The A1 revision of the FS1958 die reports its rate through a single
	 * code rather than the table above.
	 */
	if (fs19xx->devid == FS19XX_ID_FS1958 && (fs19xx->revid & 0xff) == 0xa1)
		sr = 7;

	ret = regmap_update_bits(fs19xx->regmap, FS19XX_I2SCTRL,
				 FS19XX_I2SCTRL_SR | FS19XX_I2SCTRL_DOE |
				 FS19XX_I2SCTRL_CHS12,
				 FIELD_PREP(FS19XX_I2SCTRL_SR, sr) |
				 FS19XX_I2SCTRL_DOE |
				 FIELD_PREP(FS19XX_I2SCTRL_CHS12, fs19xx->chs12));
	if (ret)
		return ret;

	/*
	 * How the part is wired to the host is the driver's business, not the
	 * tuning's. A vendor container is written for whatever topology that
	 * vendor shipped - often several of these amplifiers sharing one data
	 * line in TDM - so pin the interface back to I2S here, after the
	 * container has had its say.
	 */
	ret = regmap_update_bits(fs19xx->regmap, FS19XX_TDMCTRL,
				 FS19XX_TDMCTRL_TDMEN, 0);
	if (ret)
		return ret;

	bclk = snd_soc_params_to_bclk(params);

	return fs19xx_set_pll(fs19xx, bclk);
}

static int fs19xx_mute_stream(struct snd_soc_dai *dai, int mute, int stream)
{
	struct fs19xx_priv *fs19xx = snd_soc_component_get_drvdata(dai->component);

	return regmap_write(fs19xx->regmap, FS19XX_DACCTRL,
			    mute ? FS19XX_DACCTRL_STOP : FS19XX_DACCTRL_RUN);
}

static const struct snd_soc_dai_ops fs19xx_dai_ops = {
	.hw_params	= fs19xx_hw_params,
	.mute_stream	= fs19xx_mute_stream,
	.no_capture_mute = 1,
};

#define FS19XX_RATES	(SNDRV_PCM_RATE_8000_192000)
#define FS19XX_FORMATS	(SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE | \
			 SNDRV_PCM_FMTBIT_S32_LE)

static struct snd_soc_dai_driver fs19xx_dai = {
	.name = "fs19xx-aif",
	.playback = {
		.stream_name	= "Playback",
		.channels_min	= 1,
		.channels_max	= 2,
		.rates		= FS19XX_RATES,
		.formats	= FS19XX_FORMATS,
	},
	.ops = &fs19xx_dai_ops,
};

/*
 * 0xff is 0 dB and every step is 0.375 dB, so the register bottoms out at
 * -95.6 dB rather than at a true mute.
 */
static const DECLARE_TLV_DB_SCALE(fs19xx_vol_tlv, -9562, 37, 0);

static const struct snd_kcontrol_new fs19xx_controls[] = {
	SOC_SINGLE_TLV("Speaker Volume", FS19XX_AUDIOCTRL, 8, 0xff, 0,
		       fs19xx_vol_tlv),
};

static int fs19xx_spk_event(struct snd_soc_dapm_widget *w,
			    struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);
	struct fs19xx_priv *fs19xx = snd_soc_component_get_drvdata(component);
	int ret;

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		ret = regmap_write(fs19xx->regmap, FS19XX_SYSCTRL,
				   FS19XX_SYSCTRL_ON);
		ret = ret ?: regmap_write(fs19xx->regmap, FS19XX_PWRCTRL, 0);
		if (ret)
			return ret;
		/* Let the boost converter and the PLL settle. */
		fsleep(10000);
		return regmap_update_bits(fs19xx->regmap, FS19XX_TSCTRL,
					  FS19XX_TSCTRL_TSEN,
					  FS19XX_TSCTRL_TSEN);
	case SND_SOC_DAPM_POST_PMD:
		ret = regmap_write(fs19xx->regmap, FS19XX_PWRCTRL,
				   FS19XX_PWRCTRL_PWDN);
		if (ret)
			return ret;
		/* The ramp down has to finish before the blocks are gated. */
		fsleep(35000);
		return regmap_write(fs19xx->regmap, FS19XX_SYSCTRL, 0);
	}

	return 0;
}

static const struct snd_soc_dapm_widget fs19xx_widgets[] = {
	SND_SOC_DAPM_OUT_DRV_E("SPK PGA", SND_SOC_NOPM, 0, 0, NULL, 0,
			       fs19xx_spk_event,
			       SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_OUTPUT("SPK"),
};

static const struct snd_soc_dapm_route fs19xx_routes[] = {
	{ "SPK PGA", NULL, "Playback" },
	{ "SPK", NULL, "SPK PGA" },
};

static const struct snd_soc_component_driver fs19xx_component = {
	.controls		= fs19xx_controls,
	.num_controls		= ARRAY_SIZE(fs19xx_controls),
	.dapm_widgets		= fs19xx_widgets,
	.num_dapm_widgets	= ARRAY_SIZE(fs19xx_widgets),
	.dapm_routes		= fs19xx_routes,
	.num_dapm_routes	= ARRAY_SIZE(fs19xx_routes),
	.idle_bias_on		= 1,
	.endianness		= 1,
};

static bool fs19xx_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case FS19XX_STATUS:
	case FS19XX_CHIPINI:
	case FS19XX_PWRCTRL:
	case FS19XX_TSCTRL:
		return true;
	default:
		return false;
	}
}

static const struct regmap_config fs19xx_regmap_config = {
	.reg_bits		= 8,
	.val_bits		= 16,
	.val_format_endian	= REGMAP_ENDIAN_BIG,
	.max_register		= 0xff,
	.volatile_reg		= fs19xx_volatile_reg,
	.cache_type		= REGCACHE_NONE,
};

static int fs19xx_i2c_probe(struct i2c_client *i2c)
{
	struct device *dev = &i2c->dev;
	struct fs19xx_priv *fs19xx;
	unsigned int val;
	u32 chs12;
	int ret;

	fs19xx = devm_kzalloc(dev, sizeof(*fs19xx), GFP_KERNEL);
	if (!fs19xx)
		return -ENOMEM;

	fs19xx->dev = dev;
	i2c_set_clientdata(i2c, fs19xx);

	fs19xx->regmap = devm_regmap_init_i2c(i2c, &fs19xx_regmap_config);
	if (IS_ERR(fs19xx->regmap))
		return dev_err_probe(dev, PTR_ERR(fs19xx->regmap),
				     "failed to init regmap\n");

	fs19xx->chs12 = FS19XX_CHS12_MONO;
	if (!device_property_read_u32(dev, "foursemi,i2s-channel", &chs12)) {
		if (chs12 < FS19XX_CHS12_LEFT || chs12 > FS19XX_CHS12_MONO)
			return dev_err_probe(dev, -EINVAL,
					     "bad foursemi,i2s-channel %u\n",
					     chs12);
		fs19xx->chs12 = chs12;
	}

	/*
	 * Take the part through a full shutdown even if the boot firmware
	 * left it running: driving the pin to the level it already has does
	 * not restart the internal supplies, and the part then never answers
	 * on I2C.
	 */
	fs19xx->reset = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(fs19xx->reset))
		return dev_err_probe(dev, PTR_ERR(fs19xx->reset),
				     "failed to get the reset GPIO\n");

	if (fs19xx->reset) {
		fsleep(10000);
		gpiod_set_value_cansleep(fs19xx->reset, 0);
	}

	/* It answers about 5 ms after the pin is released. */
	fsleep(20000);

	ret = regmap_read(fs19xx->regmap, FS19XX_DEVID, &val);
	if (ret)
		return dev_err_probe(dev, ret, "failed to read the device ID\n");

	fs19xx->devid = val >> 8;
	if (fs19xx->devid != FS19XX_ID_FS1958 &&
	    fs19xx->devid != FS19XX_ID_FS19XX)
		return dev_err_probe(dev, -ENODEV, "unknown device ID %#06x\n",
				     val);

	ret = fs19xx_soft_reset(fs19xx);
	if (ret)
		return ret;

	ret = regmap_read(fs19xx->regmap, FS19XX_REVID, &fs19xx->revid);
	if (ret)
		return dev_err_probe(dev, ret, "failed to read the revision\n");

	dev_dbg(dev, "FS19xx device %#04x revision %#04x\n", fs19xx->devid,
		fs19xx->revid & 0xff);

	/*
	 * The tuning has to go in after the reset above, which is what
	 * returns the register file to its power-on values.
	 */
	ret = fs19xx_load_firmware(fs19xx, i2c->addr);
	if (ret)
		return ret;

	return devm_snd_soc_register_component(dev, &fs19xx_component,
					       &fs19xx_dai, 1);
}

static const struct i2c_device_id fs19xx_i2c_id[] = {
	{ "fs19xx" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, fs19xx_i2c_id);

static const struct of_device_id fs19xx_of_match[] = {
	{ .compatible = "foursemi,fs19xx" },
	{ }
};
MODULE_DEVICE_TABLE(of, fs19xx_of_match);

static struct i2c_driver fs19xx_i2c_driver = {
	.driver = {
		.name = "fs19xx",
		.of_match_table = fs19xx_of_match,
	},
	.probe = fs19xx_i2c_probe,
	.id_table = fs19xx_i2c_id,
};
module_i2c_driver(fs19xx_i2c_driver);

MODULE_AUTHOR("Junhao Xie <bigfoot@radxa.com>");
MODULE_DESCRIPTION("FourSemi FS19xx smart speaker amplifier driver");
MODULE_LICENSE("GPL");
