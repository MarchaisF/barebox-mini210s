#include <common.h>
#include <clock.h>
#include <driver.h>
#include <io.h>
#include <malloc.h>
#include <param.h>
#include <linux/bitops.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/rawnand.h>
#include <linux/sizes.h>
#include <mach/s3c-clocks.h>
#include <mach/s3c-iomap.h>

#define S5PV210_NFCONF			0x00000
#define S5PV210_NFCONT			0x00004
#define S5PV210_NFCMD			0x00008
#define S5PV210_NFADDR			0x0000c
#define S5PV210_NFDATA			0x00010
#define S5PV210_NFSTAT			0x00028
#define S5PV210_NFECCCONF		0x20000
#define S5PV210_NFECCCONT		0x20020
#define S5PV210_NFECCSTAT		0x20030
#define S5PV210_NFECCSECSTAT		0x20040
#define S5PV210_NFECCERL		0x200c0
#define S5PV210_NFECCERP		0x200f0
#define S5PV210_NFCONT_ENABLE		BIT(0)
#define S5PV210_NFCONT_CS0		BIT(1)
#define S5PV210_NFCONT_CS_MASK		(BIT(1) | BIT(2) | BIT(22) | BIT(23))
#define S5PV210_NFCONT_ECC_LOCK	BIT(7)
#define S5PV210_NFSTAT_READY		BIT(0)
#define S5PV210_NFSTAT_CLEAR		(BIT(4) | BIT(5))
#define S5PV210_NFCONF_EXT_ECC		(BIT(23) | BIT(24))
#define S5PV210_NFCONF_MLC		BIT(3)
#define S5PV210_NFCONF_PAGE_2K		BIT(2)
#define S5PV210_NFECCCONT_INIT		BIT(2)
#define S5PV210_NFECCCONT_ENCODE	BIT(16)
#define S5PV210_NFECCSTAT_FREE		BIT(8)
#define S5PV210_NFECCSTAT_DECODE_DONE	BIT(24)
#define S5PV210_NFECCSTAT_ENCODE_DONE	BIT(25)
#define S5PV210_NFECCSTAT_BUSY		BIT(31)
#define S5PV210_NAND_CLOCK_GATE	(BIT(28) | BIT(24))
#define S5PV210_NAND_TIMING_MASK	0xfff0
#define S5PV210_NAND_TIMING_SLOW	0xfff0

#define S5PV210_NAND_PAGE_SIZE		8192
#define S5PV210_NAND_OOB_SIZE		640
#define S5PV210_NAND_ECC_STEP		512
#define S5PV210_NAND_ECC_SLOT		28
#define S5PV210_NAND_ECC_PARITY		26
#define S5PV210_NAND_ECC_STRENGTH	16
#define S5PV210_NAND_ECC_OFFSET		36
#define S5PV210_NAND_META_OFFSET	4
#define S5PV210_NAND_META_SIZE		32
#define S5PV210_NAND_META_ECC		484
#define S5PV210_NAND_SAMSUNG_ID	0xec
#define S5PV210_NAND_DEVICE_ID		0xd7

struct s5pv210_nand {
	struct nand_chip chip;
	struct nand_controller controller;
	struct device *dev;
	void __iomem *regs;
	bool command_blocked;
	bool reading_id;
	unsigned int readid_commands;
	unsigned int read_bytes;
	u32 readid_nfcont;
	u32 readid_nfstat;
};

static bool s5pv210_read_command(int command)
{
	switch (command) {
	case NAND_CMD_READ0:
	case NAND_CMD_READ1:
	case NAND_CMD_READOOB:
	case NAND_CMD_READSTART:
	case NAND_CMD_RNDOUT:
	case NAND_CMD_RNDOUTSTART:
	case NAND_CMD_READID:
	case NAND_CMD_STATUS:
	case NAND_CMD_PARAM:
	case NAND_CMD_GET_FEATURES:
	case NAND_CMD_RESET:
		return true;
	default:
		return false;
	}
}

static void s5pv210_cmd_ctrl(struct nand_chip *chip, int command,
			     unsigned int control)
{
	struct s5pv210_nand *host = chip->priv;

	if (command == NAND_CMD_NONE)
		return;

	if (control & NAND_CLE) {
		host->command_blocked = !s5pv210_read_command(command);
		host->reading_id = command == NAND_CMD_READID;
		if (host->command_blocked) {
			dev_err(host->dev, "refusing NAND command 0x%02x\n", command);
			return;
		}
		if (command == NAND_CMD_READID)
			host->readid_commands++;
		writeb(command, host->regs + S5PV210_NFCMD);
	} else if ((control & NAND_ALE) && !host->command_blocked) {
		writeb(command, host->regs + S5PV210_NFADDR);
	}
}

static u8 s5pv210_read_byte(struct nand_chip *chip)
{
	struct s5pv210_nand *host = chip->priv;

	host->read_bytes++;
	if (host->reading_id) {
		host->readid_nfcont = readl(host->regs + S5PV210_NFCONT);
		host->readid_nfstat = readl(host->regs + S5PV210_NFSTAT);
	}
	return readb(host->regs + S5PV210_NFDATA);
}

static void s5pv210_write_forbidden(struct nand_chip *chip, const u8 *buf,
				    int length)
{
	struct s5pv210_nand *host = chip->priv;

	dev_err(host->dev, "refusing NAND data write\n");
}

static void s5pv210_select_chip(struct nand_chip *chip, int target)
{
	struct s5pv210_nand *host = chip->priv;
	u32 value = readl(host->regs + S5PV210_NFCONT);

	value |= S5PV210_NFCONT_CS_MASK;
	if (target == 0)
		value &= ~S5PV210_NFCONT_CS0;

	writel(value, host->regs + S5PV210_NFCONT);
}

static int s5pv210_dev_ready(struct nand_chip *chip)
{
	struct s5pv210_nand *host = chip->priv;

	return !!(readl(host->regs + S5PV210_NFSTAT) & S5PV210_NFSTAT_READY);
}

static void s5pv210_controller_init(struct s5pv210_nand *host)
{
	u32 value;

	value = readl(S5P_CLK_GATE_IP1) | S5PV210_NAND_CLOCK_GATE;
	writel(value, S5P_CLK_GATE_IP1);

	value = readl(host->regs + S5PV210_NFCONF);
	value &= ~S5PV210_NAND_TIMING_MASK;
	value |= S5PV210_NAND_TIMING_SLOW;
	writel(value, host->regs + S5PV210_NFCONF);

	value = readl(host->regs + S5PV210_NFCONT);
	value |= S5PV210_NFCONT_CS_MASK | S5PV210_NFCONT_ECC_LOCK |
		 S5PV210_NFCONT_ENABLE;
	writel(value, host->regs + S5PV210_NFCONT);
}

static int s5pv210_ooblayout_ecc(struct mtd_info *mtd, int section,
				struct mtd_oob_region *region)
{
	if (section != 0)
		return -ERANGE;

	region->offset = S5PV210_NAND_ECC_OFFSET;
	region->length = (S5PV210_NAND_PAGE_SIZE / S5PV210_NAND_ECC_STEP) *
			 S5PV210_NAND_ECC_SLOT;

	return 0;
}

static int s5pv210_ooblayout_free(struct mtd_info *mtd, int section,
				 struct mtd_oob_region *region)
{
	if (section != 0)
		return -ERANGE;

	region->offset = S5PV210_NAND_META_OFFSET;
	region->length = S5PV210_NAND_META_SIZE;

	return 0;
}

static const struct mtd_ooblayout_ops s5pv210_ooblayout_ops = {
	.ecc = s5pv210_ooblayout_ecc,
	.free = s5pv210_ooblayout_free,
};

static int s5pv210_apply_errors(u8 *data, unsigned int length,
			      unsigned int count, const u32 *locations,
			      const u32 *patterns)
{
	unsigned int index, position;
	u8 pattern;

	if (count > S5PV210_NAND_ECC_STRENGTH)
		return -EBADMSG;

	for (index = 0; index < count; index++) {
		position = (locations[index / 2] >> ((index % 2) * 16)) & 0x3ff;
		pattern = patterns[index / 4] >> ((index % 4) * 8);
		if (position < length)
			data[position] ^= pattern;
	}

	return count;
}

static int s5pv210_decode_chunk(struct nand_chip *chip, unsigned int column,
				u8 *data, unsigned int length, u8 *parity)
{
	struct s5pv210_nand *host = chip->priv;
	u32 locations[8] = { 0 }, patterns[4] = { 0 };
	u32 value, status, count;
	uint64_t start;
	unsigned int index;
	int ret;

	ret = nand_change_read_column_op(chip, column, NULL, 0, false);
	if (ret)
		return ret;

	value = readl(host->regs + S5PV210_NFCONF) | S5PV210_NFCONF_EXT_ECC;
	writel(value, host->regs + S5PV210_NFCONF);
	writel(((length - 1) << 16) | 5, host->regs + S5PV210_NFECCCONF);

	value = readl(host->regs + S5PV210_NFECCCONT);
	value &= ~S5PV210_NFECCCONT_ENCODE;
	writel(value, host->regs + S5PV210_NFECCCONT);
	writel(S5PV210_NFSTAT_CLEAR, host->regs + S5PV210_NFSTAT);
	writel(S5PV210_NFECCSTAT_DECODE_DONE | S5PV210_NFECCSTAT_ENCODE_DONE,
	       host->regs + S5PV210_NFECCSTAT);

	value = readl(host->regs + S5PV210_NFCONT);
	writel(value & ~S5PV210_NFCONT_ECC_LOCK, host->regs + S5PV210_NFCONT);
	value = readl(host->regs + S5PV210_NFECCCONT);
	writel(value | S5PV210_NFECCCONT_INIT, host->regs + S5PV210_NFECCCONT);

	ret = nand_read_data_op(chip, data, length, false, false);
	if (!ret) {
		for (index = 0; index < S5PV210_NAND_ECC_SLOT; index++)
			writeb(parity[index], host->regs + S5PV210_NFDATA);
	}

	value = readl(host->regs + S5PV210_NFCONT);
	writel(value | S5PV210_NFCONT_ECC_LOCK, host->regs + S5PV210_NFCONT);
	if (ret)
		return ret;

	start = get_time_ns();
	do {
		status = readl(host->regs + S5PV210_NFECCSTAT);
		if ((status & S5PV210_NFECCSTAT_DECODE_DONE) &&
		    !(status & S5PV210_NFECCSTAT_BUSY))
			break;
	} while (!is_timeout(start, 100 * MSECOND));

	if (!(status & S5PV210_NFECCSTAT_DECODE_DONE) ||
	    (status & S5PV210_NFECCSTAT_BUSY))
		return -ETIMEDOUT;

	writel(S5PV210_NFECCSTAT_DECODE_DONE, host->regs + S5PV210_NFECCSTAT);
	count = readl(host->regs + S5PV210_NFECCSECSTAT) & 0x1f;

	if ((status & S5PV210_NFECCSTAT_FREE) ||
	    count > S5PV210_NAND_ECC_STRENGTH) {
		ret = nand_check_erased_ecc_chunk(data, length, parity,
				S5PV210_NAND_ECC_PARITY, NULL, 0,
				S5PV210_NAND_ECC_STRENGTH);
		if (ret >= 0)
			return ret;
	}

	if (count > S5PV210_NAND_ECC_STRENGTH)
		return -EBADMSG;

	for (index = 0; index < (count + 1) / 2; index++)
		locations[index] = readl(host->regs + S5PV210_NFECCERL + index * 4);
	for (index = 0; index < (count + 3) / 4; index++)
		patterns[index] = readl(host->regs + S5PV210_NFECCERP + index * 4);

	return s5pv210_apply_errors(data, length, count, locations, patterns);
}

static int s5pv210_read_raw_oob(struct nand_chip *chip, int page)
{
	struct mtd_info *mtd = nand_to_mtd(chip);
	int ret;

	ret = nand_read_oob_op(chip, page, 0, chip->oob_poi, mtd->oobsize);
	if (ret)
		return ret;

	return s5pv210_dev_ready(chip) ? 0 : -ETIMEDOUT;
}

static int s5pv210_read_raw_page(struct nand_chip *chip, u8 *buf,
				 int oob_required, int page)
{
	int ret;

	ret = nand_read_page_raw(chip, buf, oob_required, page);
	if (ret)
		return ret;

	return s5pv210_dev_ready(chip) ? 0 : -ETIMEDOUT;
}

static int s5pv210_decode_metadata(struct nand_chip *chip)
{
	struct mtd_info *mtd = nand_to_mtd(chip);

	return s5pv210_decode_chunk(chip, mtd->writesize + S5PV210_NAND_META_OFFSET,
		chip->oob_poi + S5PV210_NAND_META_OFFSET,
		S5PV210_NAND_META_SIZE, chip->oob_poi + S5PV210_NAND_META_ECC);
}

static int s5pv210_read_oob(struct nand_chip *chip, int page)
{
	struct mtd_info *mtd = nand_to_mtd(chip);
	int ret;

	ret = s5pv210_read_raw_oob(chip, page);
	if (ret)
		return ret;

	ret = s5pv210_decode_metadata(chip);
	if (ret == -EBADMSG)
		mtd->ecc_stats.failed++;
	else if (ret >= 0)
		mtd->ecc_stats.corrected += ret;

	return ret;
}

static int s5pv210_read_page(struct nand_chip *chip, u8 *buf,
			     int oob_required, int page)
{
	struct mtd_info *mtd = nand_to_mtd(chip);
	unsigned int index, max_bitflips = 0;
	int ret;

	ret = s5pv210_read_raw_oob(chip, page);
	if (ret)
		return ret;

	ret = s5pv210_decode_metadata(chip);
	if (ret == -EBADMSG) {
		mtd->ecc_stats.failed++;
	} else if (ret < 0) {
		return ret;
	} else {
		mtd->ecc_stats.corrected += ret;
		max_bitflips = ret;
	}

	for (index = 0; index < S5PV210_NAND_PAGE_SIZE / S5PV210_NAND_ECC_STEP;
	     index++) {
		ret = s5pv210_decode_chunk(chip, index * S5PV210_NAND_ECC_STEP,
			buf + index * S5PV210_NAND_ECC_STEP, S5PV210_NAND_ECC_STEP,
			chip->oob_poi + S5PV210_NAND_ECC_OFFSET +
			index * S5PV210_NAND_ECC_SLOT);
		if (ret == -EBADMSG) {
			mtd->ecc_stats.failed++;
		} else if (ret < 0) {
			return ret;
		} else {
			mtd->ecc_stats.corrected += ret;
			if ((unsigned int)ret > max_bitflips)
				max_bitflips = ret;
		}
	}

	return max_bitflips;
}

static int s5pv210_block_bad(struct nand_chip *chip, loff_t offset)
{
	struct mtd_info *mtd = nand_to_mtd(chip);
	u8 marker;
	int page, ret;

	offset &= ~((loff_t)mtd->erasesize - 1);
	page = offset >> chip->page_shift;
	ret = nand_read_oob_op(chip, page, 0, &marker, 1);
	if (ret)
		return ret;
	if (!s5pv210_dev_ready(chip))
		return -ETIMEDOUT;
	if (hweight8(marker) < chip->badblockbits)
		return 1;

	page += mtd->erasesize / mtd->writesize - 1;
	ret = nand_read_oob_op(chip, page, 0, &marker, 1);
	if (ret)
		return ret;
	if (!s5pv210_dev_ready(chip))
		return -ETIMEDOUT;

	return hweight8(marker) < chip->badblockbits;
}

static int s5pv210_write_page_forbidden(struct nand_chip *chip, const u8 *buf,
					int oob_required, int page)
{
	return -EROFS;
}

static int s5pv210_write_oob_forbidden(struct nand_chip *chip, int page)
{
	return -EROFS;
}

static int s5pv210_markbad_forbidden(struct nand_chip *chip, loff_t offset)
{
	return -EROFS;
}

static int s5pv210_attach_chip(struct nand_chip *chip)
{
	struct s5pv210_nand *host = chip->priv;
	struct nand_memory_organization *memorg = nanddev_get_memorg(&chip->base);
	struct mtd_info *mtd = nand_to_mtd(chip);
	u32 value;

	if (chip->id.data[0] != S5PV210_NAND_SAMSUNG_ID ||
	    chip->id.data[1] != S5PV210_NAND_DEVICE_ID || nand_is_slc(chip) ||
	    mtd->writesize != S5PV210_NAND_PAGE_SIZE ||
	    mtd->oobsize != S5PV210_NAND_OOB_SIZE ||
	    mtd->erasesize != SZ_1M || mtd->size != SZ_4G ||
	    memorg->pages_per_eraseblock != 128) {
		dev_err(host->dev, "unsupported NAND ID %02x %02x %02x %02x %02x %02x\n",
			chip->id.data[0], chip->id.data[1], chip->id.data[2],
			chip->id.data[3], chip->id.data[4], chip->id.data[5]);
		dev_err(host->dev, "size %llu, page %u, OOB %u\n",
			(unsigned long long)mtd->size, mtd->writesize, mtd->oobsize);
		dev_err(host->dev, "erase %u, pages/block %u\n",
			mtd->erasesize, memorg->pages_per_eraseblock);
		return -ENODEV;
	}

	value = readl(host->regs + S5PV210_NFCONF);
	value &= ~S5PV210_NFCONF_PAGE_2K;
	value |= S5PV210_NFCONF_MLC | S5PV210_NFCONF_EXT_ECC;
	writel(value, host->regs + S5PV210_NFCONF);

	chip->badblockbits = 4;
	chip->ecc.mode = NAND_ECC_HW;
	chip->ecc.algo = NAND_ECC_ALGO_BCH;
	chip->ecc.size = S5PV210_NAND_ECC_STEP;
	chip->ecc.bytes = S5PV210_NAND_ECC_SLOT;
	chip->ecc.strength = S5PV210_NAND_ECC_STRENGTH;
	chip->ecc.read_page = s5pv210_read_page;
	chip->ecc.read_page_raw = s5pv210_read_raw_page;
	chip->ecc.read_oob = s5pv210_read_oob;
	chip->ecc.read_oob_raw = s5pv210_read_raw_oob;
	chip->ecc.write_page = s5pv210_write_page_forbidden;
	chip->ecc.write_page_raw = s5pv210_write_page_forbidden;
	chip->ecc.write_oob = s5pv210_write_oob_forbidden;
	chip->ecc.write_oob_raw = s5pv210_write_oob_forbidden;
	mtd_set_ooblayout(mtd, &s5pv210_ooblayout_ops);

	return 0;
}

static const struct nand_controller_ops s5pv210_controller_ops = {
	.attach_chip = s5pv210_attach_chip,
};

static int s5pv210_nand_probe(struct device *dev)
{
	struct s5pv210_nand *host;
	struct nand_chip *chip;
	struct mtd_info *mtd;
	struct resource *resource;
	int ret;

	host = kzalloc(sizeof(*host), GFP_KERNEL);
	if (!host)
		return -ENOMEM;

	resource = dev_request_mem_resource(dev, 0);
	if (IS_ERR(resource)) {
		ret = PTR_ERR(resource);
		goto free_host;
	}

	host->dev = dev;
	host->regs = IOMEM(resource->start);
	chip = &host->chip;
	chip->priv = host;
	nand_controller_init(&host->controller);
	host->controller.ops = &s5pv210_controller_ops;
	chip->controller = &host->controller;
	chip->options = NAND_ROM | NAND_SKIP_BBTSCAN | NAND_KEEP_TIMINGS |
			NAND_NO_SUBPAGE_WRITE | NAND_WAIT_TCCS;
	chip->legacy.IO_ADDR_R = host->regs + S5PV210_NFDATA;
	chip->legacy.IO_ADDR_W = host->regs + S5PV210_NFDATA;
	chip->legacy.cmd_ctrl = s5pv210_cmd_ctrl;
	chip->legacy.select_chip = s5pv210_select_chip;
	chip->legacy.dev_ready = s5pv210_dev_ready;
	chip->legacy.read_byte = s5pv210_read_byte;
	chip->legacy.write_buf = s5pv210_write_forbidden;
	chip->legacy.block_bad = s5pv210_block_bad;
	chip->legacy.block_markbad = s5pv210_markbad_forbidden;
	chip->legacy.chip_delay = 50;

	mtd = nand_to_mtd(chip);
	mtd->dev.parent = dev;
	mtd->no_auto_bb = true;
	s5pv210_controller_init(host);

	ret = nand_scan(chip, 1);
	if (ret) {
		dev_err(dev, "scan failed %d\n", ret);
		dev_err(dev, "ID: %02x %02x %02x %02x %02x %02x %02x %02x\n",
			chip->id.data[0], chip->id.data[1], chip->id.data[2],
			chip->id.data[3], chip->id.data[4], chip->id.data[5],
			chip->id.data[6], chip->id.data[7]);
		dev_err(dev, "READID commands %u, bytes read %u\n",
			host->readid_commands, host->read_bytes);
		dev_err(dev, "READID NFCONT %08x NFSTAT %08x\n",
			host->readid_nfcont, host->readid_nfstat);
		dev_err(dev, "NFCONF %08x NFCONT %08x\n",
			readl(host->regs + S5PV210_NFCONF),
			readl(host->regs + S5PV210_NFCONT));
		dev_err(dev, "NFSTAT %08x CLK_GATE_IP1 %08x\n",
			readl(host->regs + S5PV210_NFSTAT),
			readl(S5P_CLK_GATE_IP1));
		dev_err(dev, "MP0_1CON %08x MP0_3CON %08x\n",
			readl(S3C_GPIO_BASE + 0x2e0),
			readl(S3C_GPIO_BASE + 0x320));
		dev_err(dev, "MP0_6CON %08x\n", readl(S3C_GPIO_BASE + 0x380));
		goto free_host;
	}

	if (mtd->flags & MTD_WRITEABLE) {
		ret = -EROFS;
		goto cleanup_nand;
	}

	ret = add_mtd_nand_device(mtd, "nand");
	if (ret)
		goto cleanup_nand;

	dev_add_param_uint32_ro(&mtd->dev, "ecc.corrected",
				&mtd->ecc_stats.corrected, "%u");
	dev_add_param_uint32_ro(&mtd->dev, "ecc.failed",
				&mtd->ecc_stats.failed, "%u");

	dev_info(dev, "ID %02x %02x %02x %02x %02x %02x, read-only BCH16/512\n",
		 chip->id.data[0], chip->id.data[1], chip->id.data[2],
		 chip->id.data[3], chip->id.data[4], chip->id.data[5]);
	return 0;

cleanup_nand:
	nand_cleanup(chip);
free_host:
	free(host);
	return ret;
}

static struct driver s5pv210_nand_driver = {
	.name = "s5pv210-nand",
	.probe = s5pv210_nand_probe,
};
device_platform_driver(s5pv210_nand_driver);