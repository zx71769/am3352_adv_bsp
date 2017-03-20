#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/cdev.h>
#include <linux/watchdog.h>
#include <linux/fs.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <asm-generic/gpio.h>

#define DRIVER_NAME "adv_am335x_wdt"

#define AM335X_CM_PER_GPIO1_CLKCTRL (0x44E00000+0xAC)
#define AM335X_CM_PER_GPIO2_CLKCTRL (0x44E00000+0xB0)
#define AM335X_CM_PER_GPIO3_CLKCTRL (0x44E00000+0xB4)
#define AM335X_PRCM_MEM_SIZE	0x04

#define ADV_GPIO_OE				0x134	/* Output/Input enable */
#define ADV_GPIO_DATAIN			0x138	/* the data of input pin */
#define ADV_GPIO_DATAOUT		0x13C	/* the data of output pin */
#define ADV_GPIO_CLEARDATAOUT	0x190	/* Set to 0 an output */
#define ADV_GPIO_SETDATAOUT		0x194	/* Set to 1 an output */

#define ADV_GPIO0_BASE 0x44E07000
#define ADV_GPIO1_BASE 0x4804C000
#define ADV_GPIO2_BASE 0x481AC000
#define ADV_GPIO3_BASE 0x481AE000
#define ADV_GPIOPAD_MEM_SIZE 0x2000

#define ADV_WDT_OFFSET  0x00400000  /* GPIO2_22 */

int wd_enable = 1;
int wd_count = 40;
struct timer_list adv_timer;
static void __iomem *gpio_base; 
static void __iomem *prcm_base; 

static inline void _delay(void)
{
	int i = 1000;
	do{}while(i--);
}

static void adv_wdt_do_reset(void)
{
	u32 tmp;
	
	tmp = (readl(gpio_base+ADV_GPIO_DATAOUT));
	pr_info("before set: %d\n", (unsigned int)tmp);

/*
	iowrite32be(ADV_WDT_OFFSET, (__volatile__ void *)(ADV_GPIO2_BASE+ADV_GPIO_CLEARDATAOUT)); //set 0

	_delay();	

	tmp = (ioread32be((__volatile__ void *)(ADV_GPIO2_BASE+ADV_GPIO_DATAOUT))&ADV_WDT_OFFSET);
	pr_info("set 0: %d\n", tmp);
	iowrite32be(ADV_WDT_OFFSET, (__volatile__ void *)(ADV_GPIO2_BASE+ADV_GPIO_SETDATAOUT)); //set 1

	_delay();

	tmp = (ioread32be((__volatile__ void *)(ADV_GPIO2_BASE+ADV_GPIO_DATAOUT))&ADV_WDT_OFFSET);
	pr_info("set 1: %d\n", tmp);
*/
}

static void adv_am335x_wdt_reset_handle(unsigned long data)
{
	if(wd_enable){
		adv_wdt_do_reset();
	}else{
		if(wd_count > 0){
			wd_count--;
			adv_wdt_do_reset();
		}
	}
	mod_timer(&adv_timer, jiffies+(HZ/4));
}

void adv_am335x_wdt_disable(void)
{
	wd_enable = 0;
}

static long adv_am335x_wdt_ioctl(struct file *file,
								unsigned int cmd, 
								unsigned long arg)
{
	pr_info("%s\n", __func__);
	wd_count = 40;	
	return 0;
}

static int adv_am335x_wdt_open(struct inode *inode, struct file *file)
{
	pr_info("%s\n", __func__);
	wd_count = 40;

	return 0;
}

static int adv_am335x_wdt_release(struct inode *inode, struct file *file)
{
	pr_info("%s\n", __func__);
	wd_count = 0;

	return 0;
}

static const struct of_device_id adv_am335x_wdt_dt_ids[] = {
	{ .compatible = "Advantech,adv_wdt", },
	{ }
};
MODULE_DEVICE_TABLE(of, adv_am335x_wdt_dt_ids);

static struct file_operations adv_am335x_wdt_ops = {
	.owner				= THIS_MODULE,
	.open				= adv_am335x_wdt_open,
	.release			= adv_am335x_wdt_release,
	.unlocked_ioctl		= adv_am335x_wdt_ioctl,
};

static struct miscdevice adv_am335x_wdt_miscdev = {
	.minor	= WATCHDOG_MINOR,
	.name	= "adv_watchdog",
	.fops	= &adv_am335x_wdt_ops,
}; 

static int wdt_gpio_init(void)
{
	if(request_mem_region(ADV_GPIO2_BASE, 
						ADV_GPIOPAD_MEM_SIZE,
						"GPIO2_PAD") == NULL)
	{
		pr_err("adv watchdog gpio mem request failed!");
		goto gpio_req_mem_err;
	}
	gpio_base = ioremap_nocache(ADV_GPIO2_BASE, ADV_GPIOPAD_MEM_SIZE);
	if(gpio_base == NULL){
		pr_err("adv watchdog gpio memory mapping failed!\n");
		goto gpio_map_err;
	}

	if(request_mem_region(AM335X_CM_PER_GPIO2_CLKCTRL, 
								AM335X_PRCM_MEM_SIZE, 
								"GPIO2_PRCM") == NULL)
	{
		pr_err("adv watchdog prcm mem request failed!");
		goto prcm_req_mem_err;
	}
	prcm_base = ioremap_nocache(AM335X_CM_PER_GPIO2_CLKCTRL, 
										AM335X_PRCM_MEM_SIZE);
	if(prcm_base == NULL){
		pr_err("adv watchdog gpio prcm memory mapping failed!\n");
		goto prcm_map_err;
	}
	
	writel(0x02, prcm_base);
	writel(0xFFFFFFFF&~(ADV_WDT_OFFSET), gpio_base+ADV_GPIO_OE);
	pr_info("adv watchdog %s success !\n", __func__);
	
	return 0;

prcm_map_err:
	iounmap(prcm_base);
prcm_req_mem_err:
	release_mem_region(AM335X_CM_PER_GPIO2_CLKCTRL, AM335X_PRCM_MEM_SIZE);
gpio_map_err:
	iounmap(gpio_base);
gpio_req_mem_err:
	release_mem_region(ADV_GPIO2_BASE, ADV_GPIOPAD_MEM_SIZE);
	
	return -1;
}


static int __init adv_am335x_wdt_init(void)
{
	int ret;

	pr_info("Advantech am335x watchdog driver initial\n");
	
	ret = misc_register(&adv_am335x_wdt_miscdev);
	if(ret != 0){
		pr_err("register miscdev on minor=%d failed (err=%d)\n",
									WATCHDOG_MINOR, ret);
		return ret;
	}
	pr_info("register miscdev success!\n");

	if(wdt_gpio_init() == -1)
		return -EIO;

	pr_info("%s: ioremap success!\n", __func__);

	init_timer(&adv_timer);
	adv_timer.function = adv_am335x_wdt_reset_handle;
	adv_am335x_wdt_reset_handle(0);

	return ret;
}

static void __exit adv_am335x_wdt_exit(void)
{
	pr_info("%s\n", __func__);
	iounmap(prcm_base);
	release_mem_region(AM335X_CM_PER_GPIO2_CLKCTRL, AM335X_PRCM_MEM_SIZE);
	iounmap(gpio_base);
	release_mem_region(ADV_GPIO2_BASE, ADV_GPIOPAD_MEM_SIZE);
	misc_deregister(&adv_am335x_wdt_miscdev);
}

module_init(adv_am335x_wdt_init);
module_exit(adv_am335x_wdt_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Advantech ICG device server am335x series WDT driver");
