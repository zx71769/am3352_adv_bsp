#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/cdev.h>
#include <linux/watchdog.h>
#include <linux/fs.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <asm-generic/gpio.h>

#define DRIVER_NAME "adv_am335x_wdt"
#define TIMEOUT	40

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

#define ADV_WDT_OFFS  1<<22	/* GPIO2_22 */

#define READ_WDT()\
	readl(gpio_base+ADV_GPIO_DATAOUT)&ADV_WDT_OFFS
#define WRITE_WDT(val)\
do{ if(val) writel(ADV_WDT_OFFS, gpio_base+ADV_GPIO_SETDATAOUT);\
	else writel(ADV_WDT_OFFS, gpio_base+ADV_GPIO_CLEARDATAOUT); }while(0)

int wd_enable = 1;
int wd_count = TIMEOUT;
struct timer_list adv_timer;
static void __iomem *gpio_base; 
static void __iomem *prcm_base;
struct adv_wdt_driver{
	dev_t devt;
	struct cdev cdev;
	struct class *class;
	struct device *device;
};
struct adv_wdt_driver wdt; 

static void adv_wdt_do_reset(void)
{
	if(READ_WDT())
		WRITE_WDT(0);
	else
		WRITE_WDT(1);
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
	adv_am335x_wdt_disable();
	wd_count = TIMEOUT;	
	return 0;
}

static int adv_am335x_wdt_open(struct inode *inode, 
								struct file *file)
{
	wd_count = TIMEOUT;
	return 0;
}

static int adv_am335x_wdt_release(struct inode *inode, 
									struct file *file)
{
	wd_count = 0;
	return 0;
}

static struct file_operations adv_am335x_wdt_ops = {
	.owner				= THIS_MODULE,
	.open				= adv_am335x_wdt_open,
	.release			= adv_am335x_wdt_release,
	.unlocked_ioctl		= adv_am335x_wdt_ioctl,
};

static void _cdev_exit(void)
{
	cdev_del(&wdt.cdev);
	unregister_chrdev_region(wdt.devt, 1);
}

static void _class_exit(void)
{
	class_unregister(wdt.class);
	class_destroy(wdt.class);
}

static int wdt_gpio_init(void)
{
	/* 
	 * In common device-tree(am33xx-dtsi) already define gpio2_pad
	 * so pinctrl driver already create memory protech region, 
	 * we don't need to do it again
	*/
	gpio_base = ioremap_nocache(ADV_GPIO2_BASE, ADV_GPIOPAD_MEM_SIZE);
	if(gpio_base == NULL){
		pr_err("adv watchdog gpio memory mapping failed!\n");
		goto gpio_map_err;
	}

	prcm_base = ioremap_nocache(AM335X_CM_PER_GPIO2_CLKCTRL, 
										AM335X_PRCM_MEM_SIZE);
	if(prcm_base == NULL){
		pr_err("adv watchdog gpio prcm memory mapping failed!\n");
		goto prcm_map_err;
	}
	/* 
	 * Enable GPIO2 PRCM (Power, Reset and Clock Menagement)
	 * See TRM ch81.12 - CM_PER_GPIO2_CLKCTRL
	*/    	
	writel(0x02, prcm_base);
	/* 
	 * Set watchdog pin (GPIO2_22) as output 
	 * See TRM ch25.4 - GPIO_OE
	*/
	writel(0xFFFFFFFF&~(ADV_WDT_OFFS), gpio_base+ADV_GPIO_OE);
	return 0;

prcm_map_err:
	iounmap(prcm_base);
gpio_map_err:
	iounmap(gpio_base);
	return -1;
}

static int _cdev_init(void)
{
	int err;
	int major = 0;
	int minor = 0;

	wdt.devt = MKDEV(major, minor);
	if(major){
		err = register_chrdev_region(wdt.devt, 1, "adv_wdt");
	}else{
		//dynamic get driver number
		err = alloc_chrdev_region(&wdt.devt, minor, 1, "adv_wdt");
		major = MAJOR(wdt.devt);
	}
	if(err){
		pr_err("Advantech wdt get MAJOR number failed, err=%d\n", err);
		return err;
	}
	
	cdev_init(&wdt.cdev, &adv_am335x_wdt_ops);
	wdt.cdev.owner = THIS_MODULE;
	wdt.cdev.ops = &adv_am335x_wdt_ops;
	err = cdev_add(&wdt.cdev, wdt.devt, 1);
	if(err){
		pr_err("Advnatech wdt cdev add failed, err=%d\n", err);
		return err;
	}
	
	return 0;	
}

static int _device_init(void)
{
	int err;
	
	wdt.class = class_create(THIS_MODULE, "adv_wdt_class");
	if(IS_ERR(wdt.class)){
		err = PTR_ERR(wdt.class);
		pr_err("Advantech wdt create class failed, err=%d\n", err);
		goto class_failed;
	}
	
	wdt.device = device_create(wdt.class, NULL, 
								wdt.devt, NULL, "adc_wdt_device");
	if(IS_ERR(wdt.device)){
		err = PTR_ERR(wdt.device);
		pr_err("Advantech wdt create device failed, err=%d\n", err);
		goto device_failed;
	}
	return 0;

device_failed:
	_class_exit();
class_failed:
	_cdev_exit();
	return err;
}

static int __init adv_am335x_wdt_init(void)
{
	int err;
	
	err = _cdev_init();
	if(err) return err;

	err = _device_init();
	if(err) return err;

	err = wdt_gpio_init();
	if(err) return -EIO;	

	init_timer(&adv_timer);
	adv_timer.function = adv_am335x_wdt_reset_handle;
	adv_am335x_wdt_reset_handle(0);
	
	pr_info("Advantech watchdog timer ready!\n");

	return 0;
}

static void __exit adv_am335x_wdt_exit(void)
{
	pr_info("%s\n", __func__);
	iounmap(prcm_base);
	iounmap(gpio_base);
	device_destroy(wdt.class, wdt.devt);
	_class_exit();
	_cdev_exit();
}

postcore_initcall(adv_am335x_wdt_init);
module_exit(adv_am335x_wdt_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Advantech ICG device server am335x series WDT driver");
