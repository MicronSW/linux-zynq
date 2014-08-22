#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/delay.h>
#include <linux/mtd/nand_ecc.h>
#include <linux/of_mtd.h>
#include <linux/debugfs.h>
#include <linux/mtd/nand_bch.h>

#define NS_OUTPUT_PREFIX "[micron_pnand]"

#define NS_ERR(args...) \
	do { printk(KERN_ERR NS_OUTPUT_PREFIX " error: " args); } while(0)

#define NS_INFO(args...) \
	do { printk(KERN_INFO NS_OUTPUT_PREFIX " " args); } while(0)

#ifdef CONFIG_OF
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#endif

#define LLD_DRIVER_NAMD "MICRON_LLD_NAND"

#define B1 0x00000001
#define B2 0x00000002
#define B3 0x00000008
#define B4 0x00000010

#define STATUS_READY    0x40

#define CMD_STATUS	0x70

#define NAND_CMD 		0x00
#define NAND_ADDR 		0x04
#define NAND_DATA 		0x08
#define NAND_CE_TIME 		0x0C
#define NAND_WE_TIME 		0x10
#define NAND_CLE_TIME 		0x14
#define NAND_ALE_TIME 		0x18
#define NAND_DQ_TIME 		0x1C
#define NAND_RE_TIME 		0x20
#define NAND_CYCLE_TIME 	0x24
#define NAND_STORBE_TIME 	0x28
#define NAND_CR  		0x2C
#define NAND_SR  		0x30
#define NAND_CLK 		0x34
#define NAND_DDL 		0x38
#define NAND_RD_LEN 		0x3C
#define NAND_CFG 		0x40
#define NAND_LENGTH 		0x44
#define DEVICE_VERSION          0x78


struct lld_info {
	struct nand_chip	chip;
	struct mtd_info		mtd;
	struct mtd_partition    *parts;
	struct platform_device 	*pdev;


	int (*dev_ready)(struct mtd_info *mtd);
	u32 nr_parts;
	u8 ale;		/* address line number connected to ALE */
	u8 cle;		/* address line number connected to CLE */
	u8 width;	/* buswidth */
	u8 chip_delay;

	void __iomem		*nand_base;
	void __iomem		*fpga_regs;
	unsigned long		end_cmd_pending;
	unsigned long		end_cmd;
};

static void fpga_init(struct mtd_info *mtd)
{
	struct nand_chip *nc = mtd->priv;
	writel(0x00140001, nc->IO_ADDR_W + NAND_WE_TIME);
	writel(0x00280001, nc->IO_ADDR_W + NAND_CLE_TIME);
	writel(0x00280001, nc->IO_ADDR_W + NAND_ALE_TIME);	
	writel(0x00280001, nc->IO_ADDR_W + NAND_DQ_TIME);
	writel(0x00280001, nc->IO_ADDR_W + NAND_RE_TIME);
	writel(0x00290029, nc->IO_ADDR_W + NAND_CYCLE_TIME);
	writel(0x0000014, nc->IO_ADDR_W + NAND_STORBE_TIME);
	writeb(0xff, nc->IO_ADDR_W + NAND_CMD);
	printk("<0> the device version is %04x\n",readl(nc->IO_ADDR_W + DEVICE_VERSION));	
}

static void lld_nand_cmd_ctrl(struct mtd_info *mtd, int cmd, unsigned int ctrl)
{
	struct nand_chip *nc = mtd->priv;
	if(cmd == NAND_CMD_NONE)
		return;
	if(ctrl & NAND_CLE)
		writeb(cmd, nc->IO_ADDR_W + NAND_CMD);	
	else if(ctrl & NAND_ALE)
		writeb(cmd, nc->IO_ADDR_W + NAND_ADDR);	
	else 
		return;

	if(nc->options & NAND_BUSWIDTH_16) {
		return -ENOMEM;
	}
}

static void lld_nand_read_buf(struct mtd_info *mtd, uint8_t *buf, int len)
{
	int i;
	struct nand_chip *chip = mtd->priv;
	void __iomem *io_base = chip->IO_ADDR_R;

	for(i = 0; i < len; i++)
	{
		writel(B3, chip->IO_ADDR_W + NAND_CR);
		while((readl(chip->IO_ADDR_W + NAND_SR) & B4) == 0)
		buf[i] = (uint8_t)readl(io_base);
	}
}

static void lld_nand_write_buf(struct mtd_info *mtd, uint8_t *buf, int len)
{
	int i;
	struct nand_chip *chip = mtd->priv;
	for(i = 0; i < len; i++)
		writeb(buf[i],chip->IO_ADDR_W + NAND_DATA);
}


static uint8_t lld_nand_read_byte(struct mtd_info *mtd)
{
	struct nand_chip *chip = mtd->priv;
	void __iomem *io_base = chip->IO_ADDR_R;
	writel(B3, chip->IO_ADDR_W + NAND_CR);
	while((readl(chip->IO_ADDR_W + NAND_SR) & B4) == 0)
	return (uint8_t)readl(io_base);
}	

/*
 * return 0 if the nand is busy, return 1 if ready
 */
static int lld_nand_device_ready(struct mtd_info *mtd)
{
	struct nand_chip *chip = mtd->priv;
	void __iomem *io_base = chip->IO_ADDR_R;
	return(readb(chip->IO_ADDR_W + NAND_SR) & B1);
}

#ifdef CONFIG_OF
static const struct of_device_id lldnand_of_mach[];
#endif
static int lldnand_probe(struct platform_device *pdev)
{
	int err = 0;
	int retval = -ENOMEM;
	struct lld_info *fnand;
	struct mtd_info *mtd;
	struct nand_chip *nand_chip;
	struct resource *nand_res, *lld_res;
	struct lld_platform_data *pdata = NULL;
	struct mtd_part_parser_data ppdata;
#ifdef CONFIG_OF
	const struct of_device_id *match;
	const unsigned int *prop;
#endif

	fnand = devm_kzalloc(&pdev->dev, sizeof(*fnand), GFP_KERNEL);
	if(!fnand){
		dev_err(&pdev->dev, "failed to allocate device structure.\n");
		return -ENOMEM;
	}
	fnand->pdev = pdev;
	mtd = &fnand->mtd;
	nand_chip = &fnand->chip;
	nand_res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if(nand_res == NULL) {
		err = -ENODEV;
		dev_err(&pdev->dev, "platform_get_resource for NAND failed\n");
		goto out_free_data;
	}

	nand_res = request_mem_region(nand_res->start, resource_size(nand_res), pdev->name);
	if(nand_res == NULL) {
		err = -EIO;
		dev_err(&pdev->dev, "request_mem_region for cont failed\n");
		goto out_free_data;
	}

	fnand->nand_base = ioremap(nand_res->start, resource_size(nand_res));
	if(fnand->nand_base == NULL) {
		err = -EIO;
		dev_err(&pdev->dev,"ioremap for NAND failed\n");
		goto out_release_nand_mem_region;
	}
#ifdef CONFIG_OF
	prop = of_get_property(pdev->dev.of_node, "xlnx,nand-width", NULL);
	if(prop) {
		if (be32_to_cpup(prop) == 16) {
			nand_chip->options |= NAND_BUSWIDTH_16;
		} else if (be32_to_cpup(prop) == 8) {
			nand_chip->options &= ~NAND_BUSWIDTH_16;
		} else {
			dev_info(&pdev->dev, "xlnx,nand-width not valid, using 8");
			nand_chip->options &= ~NAND_BUSWIDTH_16;
		}
	} else {
		dev_info(&pdev->dev, "xlnx,nand-width not in device tree, using 8");
		nand_chip->options &= ~NAND_BUSWIDTH_16;
	}	
#endif	
	nand_chip->priv = fnand;
	mtd->priv = nand_chip;
	mtd->owner = THIS_MODULE;
	mtd->name = "lld_nand";

	nand_chip->IO_ADDR_R = fnand->nand_base + NAND_DATA;
	nand_chip->IO_ADDR_W = fnand->nand_base;
	nand_chip->ecc.mode = NAND_ECC_NONE;	
	nand_chip->cmd_ctrl = lld_nand_cmd_ctrl;
	nand_chip->dev_ready = lld_nand_device_ready;
	nand_chip->read_buf = lld_nand_read_buf;
	nand_chip->write_buf = lld_nand_write_buf;
	nand_chip->read_byte = lld_nand_read_byte;
	nand_chip->ecc.mode = NAND_ECC_SOFT_BCH;
	platform_set_drvdata(pdev,mtd);

	fpga_init(mtd);

	retval = nand_scan_ident(mtd, 1, NULL);
	if(retval)
	{
		NS_ERR("cannot scan NAND Simulator device\n");
		if(retval > 0)
			retval = -ENXIO;
		goto no_dev;	
	}
	if(!mtd_nand_has_bch()){
		NS_ERR("BCH ECC support is disabled\n");
		retval = -EINVAL;
		goto no_dev;
	}
	int eccsteps = mtd->writesize/512;
	int eccbytes = (8*13 + 7)/8;  //get form internet
	if ((mtd->oobsize < 64) || !eccsteps) {
		NS_ERR("bch not available on small page devices\n");
		retval = -EINVAL;
		goto no_dev;
	}
	if ((eccbytes*eccsteps+2) > mtd->oobsize) {
		NS_ERR("invalid bch value %u\n", 8);
		retval = -EINVAL;
		goto no_dev;
	}
	nand_chip->ecc.size = 512;
	nand_chip->ecc.bytes = eccbytes;
	NS_INFO("using %u-bit/%u bytes BCH ECC\n", 8, nand_chip->ecc.size);
	retval = nand_scan_tail(mtd);
	if (retval) {
		NS_ERR("can't register NAND Simulator\n");
		if (retval > 0)
			retval = -ENXIO;
		goto no_dev;
	}
#if 0
	if(nand_scan(mtd,1)) {
		goto no_dev;
	}
#endif
	mtd->name = "lld_nand";
#ifdef CONFIG_OF
	ppdata.of_node = pdev->dev.of_node;
#endif
	err = mtd_device_parse_register(&fnand->mtd, NULL, &ppdata, NULL, 0);
	if(!err)
	{
		goto no_dev;
	}	

out_release_nand_mem_region:
	release_mem_region(nand_res->start, resource_size(nand_res));
out_free_data:
	kfree(fnand);
no_dev:
	return retval;
	return 0;
}

static int lldnand_remove(struct platform_device *pdev)
{
	return 0;
}



static const struct of_device_id lldnand_of_mach[] = {
	{.compatible = "xlnx,ps7-fnand-1.00.a" },
	{},
};
MODULE_DEVICE_TABLE(of, lldnand_of_mach);


static struct platform_driver lld_driver = {
	.probe		= lldnand_probe,
	.remove		= lldnand_remove,
	.suspend	= NULL,
	.resume		= NULL,
	.driver		= {
		.name	= LLD_DRIVER_NAMD,
		.owner  = THIS_MODULE,
		.of_match_table = lldnand_of_mach, 
	},
};

static void __init lld_nand_init(void)
{
	return platform_driver_register(&lld_driver);
}

static void __exit lld_nand_exit(void)
{
	platform_driver_unregister(&lld_driver);
}

module_init(lld_nand_init);
module_exit(lld_nand_exit);
MODULE_AUTHOR("Micron, Inc.");

