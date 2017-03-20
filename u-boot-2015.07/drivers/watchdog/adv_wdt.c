#include <common.h>
#include <asm/arch/hardware.h>
#include <asm/io.h>
#include <asm/processor.h>
#include <asm/arch-am33xx/gpio.h>
#include <asm/arch-am33xx/cpu.h>

#define AM335X_CM_PER_GPIO1_CLKCTRL (PRCM_BASE+0xAC)
#define AM335X_CM_PER_GPIO2_CLKCTRL (PRCM_BASE+0xB0)
#define AM335X_CM_PER_GPIO3_CLKCTRL (PRCM_BASE+0xB4)

#define ADV_WDT_OFFSET	0x00400000	/* GPIO2_22 */

#define GPIO0_PAD_WRITE(value, offset) \
	__raw_writel(value, (AM33XX_GPIO0_BASE+offset))
#define GPIO1_PAD_WRITE(value, offset) \
	__raw_writel(value, (AM33XX_GPIO1_BASE+offset))
#define GPIO2_PAD_WRITE(value, offset) \
	__raw_writel(value, (AM33XX_GPIO2_BASE+offset))
#define GPIO3_PAD_WRITE(value, offset) \
	__raw_writel(value, (AM33XX_GPIO3_BASE+offset))

#define GPIO2_PAD_READ(offset)	\
	__raw_readl(AM33XX_GPIO2_BASE+offset)

#define  READ_WDT()	\
	GPIO2_PAD_READ(OMAP_GPIO_DATAOUT)&ADV_WDT_OFFSET

void adv_wdi_gpio_init(void)
{
	/* 
	 * Enable GPIO2 PRCM (Power, Reset and Clock Menagement)
	 * See TRM ch81.12 - CM_PER_GPIO2_CLKCTRL
	*/
	__raw_writel(0x02, AM335X_CM_PER_GPIO2_CLKCTRL);
	/* 
	 * Set watchdog pin (GPIO2_22) as output 
	 * See TRM ch25.4 - GPIO_OE
	*/
	GPIO2_PAD_WRITE(0xFFFFFFFF&~(ADV_WDT_OFFSET), OMAP_GPIO_OE);
}

void adv_wdi_clear(void)
{
	if(READ_WDT())
		GPIO2_PAD_WRITE(ADV_WDT_OFFSET, OMAP_GPIO_CLEARDATAOUT); //set wdt as 0
	else
		GPIO2_PAD_WRITE(ADV_WDT_OFFSET, OMAP_GPIO_SETDATAOUT);	//set wdt as 1
}

int init_func_adv_watchdog_reset(void)
{
	ADV_WDT_CLEANUP();
	return 0;
}


