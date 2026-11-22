// SPDX-License-Identifier: GPL-2.0
/*
 * Sinal de vida do kernel no HY300 (só para os testes de boot, sem console serial).
 *
 * Chamado no começo do setup_arch, antes de qualquer driver:
 *  - liga a ventoinha (GPIO0_B5, ativa em nível alto): se ela voltar a girar, o kernel
 *    começou a rodar. Depois de um reset quente ela fica parada até alguém ligá-la;
 *  - arma o watchdog do SoC (dw-wdt em 0xfe600000, tclk de 24 MHz) com o período máximo,
 *    2^31 ciclos, cerca de 89 s, em modo de reset direto. Nada o alimenta até o driver
 *    dw_wdt subir; se o kernel travar antes de alguém reiniciar o aparelho, o watchdog
 *    faz um reset quente e o pstore sobrevive.
 */
#include <linux/init.h>
#include <linux/io.h>
#include <linux/bits.h>
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
	hy300_arm_watchdog();
}
