/*
 * drivers/mmc/host/sdhci-tegra.c
 *
 * SDHCI-compatible driver for NVIDIA Tegra SoCs
 *
 * Copyright (c) 2009-2010, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#define NV_DEBUG 0

#include <linux/mmc/host.h>
#include <linux/mmc/sdio_func.h>

#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/irq.h>
#include <linux/mmc/card.h>
#include <linux/clk.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/bitops.h>
#include <mach/sdhci.h>
#include <mach/pinmux.h>
#include <nvodm_sdio.h>
#include "sdhci.h"

#define DRIVER_DESC "NVIDIA Tegra SDHCI compliant driver"
#define DRIVER_NAME "tegra-sdhci"
//20110109, , prevent a lot of reading and writing SD card firmware
#define SDCARD_ALWAYS_ON	1

// sunghoon.kim, 2010,11,05 , for remove CONFIG_BCM4329_GPIO_WL_RESET in defconfig [START]
#ifdef  CONFIG_BCM4329_GPIO_WL_RESET
#undef CONFIG_BCM4329_GPIO_WL_RESET
#endif
#if defined (CONFIG_MACH_STAR_MDM_C)
#define CONFIG_BCM4329_GPIO_WL_RESET 131
#else //TMUS_E
#define CONFIG_BCM4329_GPIO_WL_RESET 177
#endif
// sunghoon.kim, 2010,11,05 , for remove CONFIG_BCM4329_GPIO_WL_RESET in defconfig [END]

struct tegra_sdhci {
	struct platform_device	*pdev;
	struct clk		*clk;
	NvOdmSdioHandle		hOdmSdio;
	const struct tegra_pingroup_config *pinmux;
	int			nr_pins;
	int			gpio_cd;
	int			gpio_polarity_cd;
	int			irq_cd;
	int			gpio_wp;
	int			gpio_polarity_wp;
	unsigned int		debounce;
	unsigned long		max_clk;
	bool			card_present;
	bool			clk_enable;
	bool			card_always_on;
	u32			sdhci_ints;
//20110110, , prevent a lot of reading and writing SD card firmware [START]
#ifdef SDCARD_ALWAYS_ON
	bool			bypass_sd_reinit;
#endif
//20110110, , prevent a lot of reading and writing SD card firmware [END]
};

static inline unsigned long res_size(struct resource *res)
{
	return res->end - res->start + 1;
}

static int tegra_sdhci_enable_dma(struct sdhci_host *sdhost)
{
	return 0;
}

static irqreturn_t card_detect_isr(int irq, void *dev_id)
{
	struct sdhci_host *sdhost = dev_id;
	struct tegra_sdhci *host = sdhci_priv(sdhost);

	host->card_present =
		(gpio_get_value(host->gpio_cd) == host->gpio_polarity_cd);
	smp_wmb();
//20110124, , change interrupt service routine from direct callback to scheduler [START]
#if 0
	sdhci_card_detect_callback(sdhost);
#else
	tasklet_schedule(&sdhost->card_tasklet);
#endif
//20110124, , change interrupt service routine from direct callback to scheduler [END]

	return IRQ_HANDLED;
}

static bool tegra_sdhci_card_detect(struct sdhci_host *sdhost)
{
	struct tegra_sdhci *host = sdhci_priv(sdhost);
	smp_rmb();
	return host->card_present;
}

static void tegra_sdhci_status_notify_cb(int card_present, void *dev_id)
{
	struct sdhci_host *sdhost = dev_id;
	struct tegra_sdhci *host = sdhci_priv(sdhost);

	dev_dbg(&host->pdev->dev, "%s: card_present %d\n",
		mmc_hostname(sdhost->mmc), card_present);

	host->card_present = card_present;
	sdhci_card_detect_callback(sdhost);
}

static int tegra_sdhci_get_ro(struct sdhci_host *sdhost)
{
	struct tegra_sdhci *host = sdhci_priv(sdhost);

	BUG_ON(host->gpio_wp == -1);
	return (gpio_get_value(host->gpio_wp) == host->gpio_polarity_wp);
}

static void tegra_sdhci_set_clock(struct sdhci_host *sdhost,
	unsigned int clock)
{
	struct tegra_sdhci *host = sdhci_priv(sdhost);

	if (clock && !host->clk_enable) {
		clk_enable(host->clk);
		host->clk_enable = true;
	} else if (!clock && host->clk_enable) {
		clk_disable(host->clk);
		host->clk_enable = false;
	}

	if (clock) {
		if (clock > host->max_clk)
			clock = host->max_clk;
		clk_set_rate(host->clk, clock);

// 20111205 mingi.sung@lge.com Check SDIO clock for Wi-Fi [START]
		if(host->pdev->id == 0) {
			if( (clock == 187500 && clk_get_rate(host->clk) < 187500)
					|| (clock == 48000000 && clk_get_rate(host->clk) < 48000000) ) { 
				clk_set_rate(host->clk, clock);
				printk("[Wi-Fi] %s:%d clk_set_rate again!\n",__func__,__LINE__);
			}
		}
// 20111205 mingi.sung@lge.com Check SDIO clock for Wi-Fi [END]
		
		sdhost->max_clk = clk_get_rate(host->clk);
		dev_dbg(&host->pdev->dev, "clock request: %uKHz. currently "
			"%uKHz\n", clock/1000, sdhost->max_clk/1000);
	}
}

// 20100513  Implementing WLAN card detection procedure temporarily [START]
#if defined (CONFIG_LGE_BCM432X_PATCH)
struct sdhci_host	*g_sdhost;
void do_wifi_cardetect(void *p)
{
	struct tegra_sdhci *t_sdhci = sdhci_priv(g_sdhost);

	printk("[Wi-Fi] %s:%d\n",__func__,__LINE__);
	
	t_sdhci->card_present = gpio_get_value(CONFIG_BCM4329_GPIO_WL_RESET);

	printk("[Wi-Fi] card_present value is %d\n", t_sdhci->card_present);
#if 0
	sdhci_card_detect_callback(g_sdhost);
#else
		tasklet_schedule(&g_sdhost->card_tasklet);
#endif

}
EXPORT_SYMBOL(do_wifi_cardetect);
#endif
// 20100513  Implementing WLAN card detection procedure temporarily [END]

static struct sdhci_ops tegra_sdhci_wp_cd_ops = {
	.enable_dma		= tegra_sdhci_enable_dma,
	.get_ro			= tegra_sdhci_get_ro,
	.card_detect		= tegra_sdhci_card_detect,
	.set_clock		= tegra_sdhci_set_clock,
};

static struct sdhci_ops tegra_sdhci_cd_ops = {
	.enable_dma		= tegra_sdhci_enable_dma,
	.card_detect		= tegra_sdhci_card_detect,
	.set_clock		= tegra_sdhci_set_clock,
};

static struct sdhci_ops tegra_sdhci_wp_ops = {
	.enable_dma		= tegra_sdhci_enable_dma,
	.get_ro			= tegra_sdhci_get_ro,
	.set_clock		= tegra_sdhci_set_clock,
};

static struct sdhci_ops tegra_sdhci_ops = {
	.enable_dma		= tegra_sdhci_enable_dma,
	.set_clock		= tegra_sdhci_set_clock,
};

int __init tegra_sdhci_probe(struct platform_device *pdev)
{
	struct sdhci_host *sdhost;
	struct tegra_sdhci *host;
	struct tegra_sdhci_platform_data *plat = pdev->dev.platform_data;
	struct resource *res;
	int ret = -ENODEV;

	if (pdev->id == -1) {
		dev_err(&pdev->dev, "dynamic instance assignment not allowed\n");
		return -ENODEV;
	}

	sdhost  = sdhci_alloc_host(&pdev->dev, sizeof(struct tegra_sdhci));
	if (IS_ERR_OR_NULL(sdhost)) {
		dev_err(&pdev->dev, "unable to allocate driver structure\n");
		return (!sdhost) ? -ENOMEM : PTR_ERR(sdhost);
	}
	sdhost->hw_name = dev_name(&pdev->dev);

// 20100513  Implementing WLAN card detection procedure temporarily [START]
#if defined (CONFIG_LGE_BCM432X_PATCH)
	if(pdev->id == 0) {
		g_sdhost= sdhost;
	}
#endif
// 20100513  Implementing WLAN card detection procedure temporarily [END]

	host = sdhci_priv(sdhost);

	host->hOdmSdio = NvOdmSdioOpen(pdev->id);
	if (!host->hOdmSdio)
		dev_info(&pdev->dev, "no ODM SDIO adaptation\n");

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "no memory I/O resource provided\n");
		ret = -ENODEV;
		goto err_sdhci_alloc;
	}
	if (!request_mem_region(res->start, res_size(res),
				dev_name(&pdev->dev))) {
		dev_err(&pdev->dev, "memory in use\n");
		ret = -EBUSY;
		goto err_sdhci_alloc;
	}
	sdhost->ioaddr = ioremap(res->start, res_size(res));
	if (!sdhost->ioaddr) {
		dev_err(&pdev->dev, "failed to map registers\n");
		ret = -ENXIO;
		goto err_request_mem;
	}
	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!res) {
		dev_err(&pdev->dev, "no IRQ resource provided\n");
		ret = -ENODEV;
		goto err_ioremap;
	}
	sdhost->irq = res->start;
	host->clk = clk_get(&pdev->dev, NULL);
	if (!host->clk) {
		dev_err(&pdev->dev, "unable to get clock\n");
		ret = -ENODEV;
		goto err_ioremap;
	}

	host->pdev = pdev;
	host->pinmux = plat->pinmux;
	host->nr_pins = plat->nr_pins;
	host->gpio_cd = plat->gpio_nr_cd;
	host->gpio_polarity_cd = plat->gpio_polarity_cd;
	host->gpio_wp = plat->gpio_nr_wp;
	host->gpio_polarity_wp = plat->gpio_polarity_wp;
	host->card_always_on = plat->is_always_on;
	dev_dbg(&pdev->dev, "write protect: %d card detect: %d\n",
		host->gpio_wp, host->gpio_cd);
	host->irq_cd = -1;
	host->debounce = plat->debounce;
	if (plat->max_clk)
		host->max_clk = min_t(unsigned int, 52000000, plat->max_clk);
	else {
		dev_info(&pdev->dev, "no max_clk specified, default to 52MHz\n");
		host->max_clk = 52000000;
	}

#ifdef CONFIG_EMBEDDED_MMC_START_OFFSET
	sdhost->start_offset = plat->offset;
#endif

	if (host->gpio_cd != -1) {
		ret = gpio_request(host->gpio_cd, "card_detect");
		if (ret < 0) {
			dev_err(&pdev->dev, "request cd gpio failed\n");
			host->gpio_cd = -1;
			goto skip_gpio_cd;
		}
		host->irq_cd = gpio_to_irq(host->gpio_cd);
		if (host->irq_cd < 0) {
			/* fall back to non-GPIO card detect mode */
			dev_err(&pdev->dev, "invalid card detect GPIO\n");
			host->gpio_cd = -1;
			host->irq_cd = -1;
			goto skip_gpio_cd;
		}
		ret = gpio_direction_input(host->gpio_cd);
		if (ret < 0) {
			dev_err(&pdev->dev, "failed to configure GPIO\n");
			gpio_free(host->gpio_cd);
			host->gpio_cd = -1;
			goto skip_gpio_cd;
		}
		ret = request_irq(host->irq_cd, card_detect_isr,
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			mmc_hostname(sdhost->mmc), sdhost);
		if (ret) {
			dev_err(&pdev->dev, "unable to request IRQ\n");
			gpio_free(host->gpio_cd);
			host->gpio_cd = -1;
			host->irq_cd = -1;
			goto skip_gpio_cd;
		}
		host->card_present =
			(gpio_get_value(host->gpio_cd) ==
				host->gpio_polarity_cd);
	}
skip_gpio_cd:
	ret = 0;
	if (host->gpio_wp != -1) {
		ret = gpio_request(host->gpio_wp, "write_protect");
		if (ret < 0) {
			dev_err(&pdev->dev, "request wp gpio failed\n");
			host->gpio_wp = -1;
			goto skip_gpio_wp;
		}
		ret = gpio_direction_input(host->gpio_wp);
		if (ret < 0) {
			dev_err(&pdev->dev, "configure wp gpio failed\n");
			gpio_free(host->gpio_wp);
			host->gpio_wp = -1;
		}
	}
skip_gpio_wp:
	ret = 0;
	if (host->pinmux && host->nr_pins)
		tegra_pinmux_config_tristate_table(host->pinmux,
			host->nr_pins, TEGRA_TRI_NORMAL);
	clk_enable(host->clk);
	clk_set_rate(host->clk, host->max_clk);
	host->max_clk = clk_get_rate(host->clk);
	host->clk_enable = true;

	if (host->gpio_wp != -1 && (host->gpio_cd != -1 || !plat->is_removable))
		sdhost->ops = &tegra_sdhci_wp_cd_ops;
	else if (host->gpio_wp != -1)
		sdhost->ops = &tegra_sdhci_wp_ops;
	else if (host->gpio_cd != -1 || !plat->is_removable)
		sdhost->ops = &tegra_sdhci_cd_ops;
	else
		sdhost->ops = &tegra_sdhci_ops;

	sdhost->quirks =
		SDHCI_QUIRK_BROKEN_TIMEOUT_VAL |
		SDHCI_QUIRK_SINGLE_POWER_WRITE |
		SDHCI_QUIRK_ENABLE_INTERRUPT_AT_BLOCK_GAP |
		SDHCI_QUIRK_BROKEN_WRITE_PROTECT |
		SDHCI_QUIRK_BROKEN_CARD_DETECTION |
//20100102, jm1.lee@lge.com, disable the flag not to set clock as a zero after timeout [START]
		SDHCI_QUIRK_BROKEN_CTRL_HISPD |
		SDHCI_QUIRK_RUNTIME_DISABLE;
//		SDHCI_QUIRK_BROKEN_CTRL_HISPD;
//20100102, jm1.lee@lge.com, disable the flag not to set clock as a zero after timeout [END]
#ifdef CONFIG_ARCH_TEGRA_2x_SOC
	sdhost->quirks |= SDHCI_QUIRK_BROKEN_SPEC_VERSION |
		SDHCI_QUIRK_NO_64KB_ADMA;
	sdhost->version = SDHCI_SPEC_200;
#endif

	if (!plat->is_removable)
		host->card_present = true;

	if (plat->register_status_notify)
		plat->register_status_notify(tegra_sdhci_status_notify_cb,
			sdhost);

	sdhost->data_width = plat->bus_width;
	sdhost->dma_mask = DMA_BIT_MASK(32);
	ret = sdhci_add_host(sdhost);
	if (ret)
		goto fail;

	platform_set_drvdata(pdev, sdhost);

	dev_info(&pdev->dev, "probe complete\n");

	return  0;

fail:
	if (host->irq_cd != -1)
		free_irq(host->irq_cd, sdhost);
	if (host->gpio_cd != -1)
		gpio_free(host->gpio_cd);
	if (host->gpio_wp != -1)
		gpio_free(host->gpio_wp);

	if (host->pinmux && host->nr_pins)
		tegra_pinmux_config_tristate_table(host->pinmux,
			host->nr_pins, TEGRA_TRI_TRISTATE);

	clk_disable(host->clk);
	clk_put(host->clk);
err_ioremap:
	iounmap(sdhost->ioaddr);
err_request_mem:
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	release_mem_region(res->start, res_size(res));
err_sdhci_alloc:
	if (host->hOdmSdio)
		NvOdmSdioClose(host->hOdmSdio);
	sdhci_free_host(sdhost);
	dev_err(&pdev->dev, "probe failed\n");
	return ret;
}


static int tegra_sdhci_remove(struct platform_device *pdev)
{
	struct sdhci_host *sdhost = platform_get_drvdata(pdev);
	struct tegra_sdhci *host = sdhci_priv(sdhost);

	if (host->irq_cd != -1)
		free_irq(host->irq_cd, sdhost);

	if (host->gpio_cd != -1)
		gpio_free(host->gpio_cd);

	if (host->gpio_wp != -1)
		gpio_free(host->gpio_wp);

	if (host->pinmux && host->nr_pins)
		tegra_pinmux_config_tristate_table(host->pinmux,
			host->nr_pins, TEGRA_TRI_TRISTATE);

	if (host->clk_enable)
		clk_disable(host->clk);

	clk_put(host->clk);
	iounmap(sdhost->ioaddr);
	sdhost->ioaddr = NULL;

	if (host->hOdmSdio)
		NvOdmSdioClose(host->hOdmSdio);

	sdhci_free_host(sdhost);
	return 0;
}

#if defined(CONFIG_BRCM_LGE_WL_HOSTWAKEUP)
#include <linux/earlysuspend.h>
struct early_suspend dhdpm;
EXPORT_SYMBOL(dhdpm);

void register_mmc_card_pm(struct early_suspend *cardpm)
{
	if((cardpm != NULL) && (cardpm->suspend !=NULL) && (cardpm->resume != NULL))
	{
		dhdpm.suspend = cardpm->suspend;
		dhdpm.resume = cardpm->resume;
		printk("[sdhci-tegra]%s:%d - dhdpm callback func register OK\n",__func__,__LINE__);
	}
	else
		printk("[sdhci-tegra]%s:%d - dhdpm callback func register Fail\n",__func__,__LINE__);
}
EXPORT_SYMBOL(register_mmc_card_pm);

void unregister_mmc_card_pm(void)
{
	printk("[sdhci-tegra]%s:%d - dhdpm callback func set NULL\n",__func__,__LINE__);
	dhdpm.suspend = NULL;
	dhdpm.resume  = NULL;	
}
EXPORT_SYMBOL(unregister_mmc_card_pm);
#endif /* CONFIG_BRCM_LGE_WL_HOSTWAKEUP */

#define is_card_sdio(_card) \
((_card) && ((_card)->type == MMC_TYPE_SDIO))

//20110109, , prevent a lot of reading and writing SD card firmware [START]
#ifdef SDCARD_ALWAYS_ON
#define is_card_sd(_card) \
((_card) && ((_card)->type == MMC_TYPE_SD))
#endif
//20110109, , prevent a lot of reading and writing SD card firmware [END]

#if defined(CONFIG_PM)
#define dev_to_host(_dev) platform_get_drvdata(to_platform_device(_dev))

static void tegra_sdhci_restore_interrupts(struct sdhci_host *sdhost)
{
	u32 ierr;
	u32 clear = SDHCI_INT_ALL_MASK;
	struct tegra_sdhci *host = sdhci_priv(sdhost);

	/* enable required interrupts */
	ierr = sdhci_readl(sdhost, SDHCI_INT_ENABLE);
	ierr &= ~clear;
//20110109, , prevent a lot of reading and writing SD card firmware [START]
#ifdef SDCARD_ALWAYS_ON
	if (is_card_sdio(sdhost->mmc->card))
		ierr |= host->sdhci_ints;
	else if (is_card_sd(sdhost->mmc->card))
		ierr = (SDHCI_INT_BUS_POWER | SDHCI_INT_DATA_END_BIT |
			SDHCI_INT_DATA_CRC | SDHCI_INT_DATA_TIMEOUT | SDHCI_INT_INDEX |
			SDHCI_INT_END_BIT | SDHCI_INT_CRC | SDHCI_INT_TIMEOUT |
			SDHCI_INT_DATA_END | SDHCI_INT_RESPONSE);
#else	//original source
	ierr |= host->sdhci_ints;
#endif
//20110109, , prevent a lot of reading and writing SD card firmware [END]

	sdhci_writel(sdhost, ierr, SDHCI_INT_ENABLE);
	sdhci_writel(sdhost, ierr, SDHCI_SIGNAL_ENABLE);

//20110109, , prevent a lot of reading and writing SD card firmware [START]
#ifdef SDCARD_ALWAYS_ON
	if (is_card_sdio(sdhost->mmc->card))
	if ((host->sdhci_ints & SDHCI_INT_CARD_INT) &&
		(sdhost->quirks & SDHCI_QUIRK_ENABLE_INTERRUPT_AT_BLOCK_GAP)) {
		u8 gap_ctrl = sdhci_readb(sdhost, SDHCI_BLOCK_GAP_CONTROL);
		gap_ctrl |= 0x8;
		sdhci_writeb(sdhost, gap_ctrl, SDHCI_BLOCK_GAP_CONTROL);
	}
#else	//original source
	if ((host->sdhci_ints & SDHCI_INT_CARD_INT) &&
		(sdhost->quirks & SDHCI_QUIRK_ENABLE_INTERRUPT_AT_BLOCK_GAP)) {
		u8 gap_ctrl = sdhci_readb(sdhost, SDHCI_BLOCK_GAP_CONTROL);
		gap_ctrl |= 0x8;
		sdhci_writeb(sdhost, gap_ctrl, SDHCI_BLOCK_GAP_CONTROL);
	}
#endif
//20110109, , prevent a lot of reading and writing SD card firmware [END]
}

static int tegra_sdhci_restore(struct sdhci_host *sdhost)
{
	unsigned long timeout;
	u8 mask = SDHCI_RESET_ALL;

	sdhci_writeb(sdhost, mask, SDHCI_SOFTWARE_RESET);

	sdhost->clock = 0;

	/* Wait max 100 ms */
	timeout = 100;

	/* hw clears the bit when it's done */
	while (sdhci_readb(sdhost, SDHCI_SOFTWARE_RESET) & mask) {
		if (timeout == 0) {
			printk(KERN_ERR "%s: Reset 0x%x never completed.\n",
				mmc_hostname(sdhost->mmc), (int)mask);
			return -EIO;
		}
		timeout--;
		mdelay(1);
	}

	tegra_sdhci_restore_interrupts(sdhost);
	sdhost->last_clk = 0;
	return 0;
}

static int tegra_sdhci_suspend(struct device *dev)
{
	struct sdhci_host *sdhost = dev_to_host(dev);
	struct tegra_sdhci *host = sdhci_priv(sdhost);
	struct pm_message event = { PM_EVENT_SUSPEND };
	int ret = 0;
//20110109, , prevent a lot of reading and writing SD card firmware [START]
#ifdef SDCARD_ALWAYS_ON
	u32 ierr;

	host->bypass_sd_reinit = host->card_present;
	if (host->card_always_on) {
		if (is_card_sdio(sdhost->mmc->card)) {
			int div = 0;
			u16 clk;
			unsigned int clock = 100000;

#if defined(CONFIG_BRCM_LGE_WL_HOSTWAKEUP)
			if(dhdpm.suspend != NULL) {
				printk("[sdhci-tegra]%s:%d - call dhdpm.suspend\n",__func__,__LINE__);
				dhdpm.suspend(NULL);
			}
			else
				printk("[sdhci-tegra]%s:%d - dhdpm.suspend is NULL\n",__func__,__LINE__);
#endif /* CONFIG_BRCM_LGE_WL_HOSTWAKEUP */

			/* save interrupt status before suspending */
			host->sdhci_ints = sdhci_readl(sdhost, SDHCI_INT_ENABLE);

			/* reduce host controller clk and card clk to 100 KHz */
			tegra_sdhci_set_clock(sdhost, clock);
			sdhci_writew(sdhost, 0, SDHCI_CLOCK_CONTROL);

			if (sdhost->max_clk > clock) {
				div =  1 << (fls(sdhost->max_clk / clock) - 2);
				if (div > 128)
					div = 128;
			}

			clk = div << SDHCI_DIVIDER_SHIFT;
			clk |= SDHCI_CLOCK_INT_EN | SDHCI_CLOCK_CARD_EN;
			sdhci_writew(sdhost, clk, SDHCI_CLOCK_CONTROL);
		} else if (is_card_sd(sdhost->mmc->card) && host->card_present) {
//20110214, , cancel delayed work queue when the device enter sleep
			sdhci_cancel_delayed_work(sdhost, event);
			if(host->clk_enable) {
				clk_disable(host->clk);
				host->clk_enable = false;
			}
		}
#else	//original source
	if (host->card_always_on && is_card_sdio(sdhost->mmc->card)) {
		int div = 0;
		u16 clk;
		unsigned int clock = 100000;

#if defined(CONFIG_BRCM_LGE_WL_HOSTWAKEUP)
			if(dhdpm.suspend != NULL) {
				printk("[sdhci-tegra]%s:%d - call dhdpm.suspend\n",__func__,__LINE__);
				dhdpm.suspend(NULL);
			}
			else
				printk("[sdhci-tegra]%s:%d - dhdpm.suspend is NULL\n",__func__,__LINE__);
#endif /* CONFIG_BRCM_LGE_WL_HOSTWAKEUP */

		/* save interrupt status before suspending */
		host->sdhci_ints = sdhci_readl(sdhost, SDHCI_INT_ENABLE);

		/* reduce host controller clk and card clk to 100 KHz */
		tegra_sdhci_set_clock(sdhost, clock);
		sdhci_writew(sdhost, 0, SDHCI_CLOCK_CONTROL);

		if (sdhost->max_clk > clock) {
			div =  1 << (fls(sdhost->max_clk / clock) - 2);
			if (div > 128)
				div = 128;
		}

		clk = div << SDHCI_DIVIDER_SHIFT;
		clk |= SDHCI_CLOCK_INT_EN | SDHCI_CLOCK_CARD_EN;
		sdhci_writew(sdhost, clk, SDHCI_CLOCK_CONTROL);
#endif
//20110109, , prevent a lot of reading and writing SD card firmware [END]

		return ret;
	}

	ret = sdhci_suspend_host(sdhost, event);
	if (ret) {
		dev_err(dev, "failed to suspend host\n");
		return ret;
	}

	if (host->hOdmSdio)
		NvOdmSdioSuspend(host->hOdmSdio);

	return ret;
}

static int tegra_sdhci_resume(struct device *dev)
{
	struct sdhci_host *sdhost = dev_to_host(dev);
	struct tegra_sdhci *host = sdhci_priv(sdhost);
	int ret_tegra_sdhci_resume;

	if (!host->clk_enable) {
		clk_enable(host->clk);
		host->clk_enable = true;
	}

	if (host->gpio_cd != -1)
		host->card_present =
			(gpio_get_value(host->gpio_cd) == host->gpio_polarity_cd);

//20110109, , prevent a lot of reading and writing SD card firmware [START]
#ifdef SDCARD_ALWAYS_ON
	host->bypass_sd_reinit &= host->card_present;
	if (host->card_always_on && (is_card_sdio(sdhost->mmc->card) ||
		(is_card_sd(sdhost->mmc->card) && host->bypass_sd_reinit))) {
#else	//orignal source
	if (host->card_always_on && is_card_sdio(sdhost->mmc->card)) {
#endif
//20110109, , prevent a lot of reading and writing SD card firmware [END]
		int ret = 0;

		/* soft reset SD host controller and enable interrupts */
		ret = tegra_sdhci_restore(sdhost);
		if (ret) {
			dev_err(dev, "failed to resume host\n");
			return ret;
		}

		mmiowb();
		sdhost->mmc->ops->set_ios(sdhost->mmc, &sdhost->mmc->ios);
#if defined(CONFIG_BRCM_LGE_WL_HOSTWAKEUP)
		if(is_card_sdio(sdhost->mmc->card)) {
        if(dhdpm.resume != NULL) {
            printk("[sdhci-tegra]%s:%d - call dhdpm.resume\n",__func__,__LINE__);
            dhdpm.resume(NULL);
        }
        else
            printk("[sdhci-tegra]%s:%d - dhdpm.resume is NULL\n",__func__,__LINE__);
		}
#endif /* CONFIG_BRCM_LGE_WL_HOSTWAKEUP */
		return 0;
	}

	if (host->hOdmSdio)
		NvOdmSdioResume(host->hOdmSdio);

	ret_tegra_sdhci_resume = sdhci_resume_host(sdhost);
 
//	return sdhci_resume_host(sdhost);
	return ret_tegra_sdhci_resume;
}
static struct dev_pm_ops tegra_sdhci_pm = {
	.suspend = tegra_sdhci_suspend,
	.resume = tegra_sdhci_resume,
};
#define tegra_sdhci_pm_ops (&tegra_sdhci_pm)
#else
#define tegra_sdhci_pm_ops NULL
#endif

struct platform_driver tegra_sdhci_driver = {
	.probe		= tegra_sdhci_probe,
	.remove		= tegra_sdhci_remove,
	.driver		= {
		.name	= "tegra-sdhci",
		.owner	= THIS_MODULE,
		.pm	= tegra_sdhci_pm_ops,
	},
};

static int __init tegra_sdhci_init(void)
{
	return platform_driver_register(&tegra_sdhci_driver);
}

static void __exit tegra_sdhci_exit(void)
{
	platform_driver_unregister(&tegra_sdhci_driver);
}

module_init(tegra_sdhci_init);
module_exit(tegra_sdhci_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION(DRIVER_DESC);
