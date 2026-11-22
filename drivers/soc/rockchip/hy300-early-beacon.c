// SPDX-License-Identifier: GPL-2.0
/*
 * Sinal de vida do kernel no HY300 (só para os testes de boot, sem console serial).
 *
 * Chamado no começo do setup_arch, antes de qualquer driver:
 *  - liga a ventoinha (GPIO0_B5, ativa em nível alto): se ela voltar a girar, o kernel
 *    começou a rodar. Depois de um reset quente ela fica parada até alguém ligá-la;
 *  - faz o reset do watchdog virar reinício: no CRU_GLB_RST_CON (CRU + 0xdc, TRM do RK3568
 *    Part 1) o padrão é o watchdog disparar o "second global reset" e também resetar o PMU
 *    (bit 4 em 0), e o HY300 desligava em vez de reiniciar. Com o bit 1 o watchdog dispara o
 *    "first global reset", o mesmo do PSCI/sysrq b, que reinicia; com o bit 4 ele deixa o
 *    domínio do PMU (GPIO0, PMU_GRF com o modo de reboot) em paz. O valor antigo vai para o
 *    log;
 *  - arma o watchdog do SoC (dw-wdt em 0xfe600000, tclk de 24 MHz) com o período máximo,
 *    2^31 ciclos, cerca de 89 s, em modo de reset direto. Nada o alimenta até o driver
 *    dw_wdt subir; se o kernel travar antes de alguém reiniciar o aparelho, o watchdog
 *    faz um reset quente e o pstore sobrevive.
 */
#include <linux/init.h>
#include <linux/io.h>
#include <linux/bits.h>
#include <linux/printk.h>
#include <asm/early_ioremap.h>

#define GPIO0_BASE		0xfdd60000
#define GPIO_SWPORT_DR_L	0x00
#define GPIO_SWPORT_DDR_L	0x08
#define FAN_PIN			13	/* GPIO0_B5 */

#define WDT_BASE		0xfe600000
#define WDT_CR			0x00
#define WDT_TORR		0x04
#define WDT_CRR			0x0c
#define WDT_CR_EN		BIT(0)
#define WDT_TORR_MAX		0xff	/* TOP e TOP_INIT = 15 */
#define WDT_KICK		0x76

#define CRU_BASE		0xfdd20000
#define CRU_GLB_RST_CON		0xdc	/* sem máscara de escrita nos 16 bits altos */
#define GLB_RST_WDT_FIRST	BIT(1)	/* wdt_glb_srst_ctrl: watchdog -> first global reset */
#define GLB_RST_WDT_NO_PMU	BIT(4)	/* pmu_srst_wdt_en: o watchdog não reseta o PMU */

static u32 hy300_glb_rst_before, hy300_glb_rst_after;

/* Os registradores do GPIO v2 levam a máscara de escrita nos 16 bits altos. */
static void __init hy300_set_gpio_l(void __iomem *base, unsigned int off, unsigned int pin)
{
	writel(BIT(pin + 16) | BIT(pin), base + off);
}

static void __init hy300_fan_on(void)
{
	void __iomem *gpio = early_ioremap(GPIO0_BASE, 0x100);

	if (!gpio)
		return;
	hy300_set_gpio_l(gpio, GPIO_SWPORT_DR_L, FAN_PIN);
	hy300_set_gpio_l(gpio, GPIO_SWPORT_DDR_L, FAN_PIN);
	early_iounmap(gpio, 0x100);
}

static void __init hy300_make_watchdog_reboot(void)
{
	void __iomem *cru = early_ioremap(CRU_BASE + CRU_GLB_RST_CON, sizeof(u32));

	if (!cru)
		return;
	hy300_glb_rst_before = readl(cru);
	writel(hy300_glb_rst_before | GLB_RST_WDT_FIRST | GLB_RST_WDT_NO_PMU, cru);
	hy300_glb_rst_after = readl(cru);
	early_iounmap(cru, sizeof(u32));
}

static int __init hy300_report_glb_rst(void)
{
	pr_info("hy300: CRU_GLB_RST_CON 0x%08x -> 0x%08x (watchdog reinicia)\n",
		hy300_glb_rst_before, hy300_glb_rst_after);
	return 0;
}
late_initcall(hy300_report_glb_rst);

static void __init hy300_arm_watchdog(void)
{
	void __iomem *wdt = early_ioremap(WDT_BASE, 0x100);

	if (!wdt)
		return;
	writel(WDT_TORR_MAX, wdt + WDT_TORR);
	writel(WDT_CR_EN, wdt + WDT_CR);
	writel(WDT_KICK, wdt + WDT_CRR);
	early_iounmap(wdt, 0x100);
}

void __init hy300_early_beacon(void)
{
	hy300_fan_on();
	hy300_make_watchdog_reboot();
	hy300_arm_watchdog();
}
