// SPDX-License-Identifier: GPL-2.0
/*
 * HY300 (RK3566): o reset do watchdog do SoC vira reinício, não desligamento.
 *
 * O dw-wdt aciona o reset global configurado no CRU_GLB_RST_CON (CRU + 0xdc, TRM do
 * RK3568 Part 1). No HY300 o valor que chega do U-Boot é 0x103: o bit 1
 * (wdt_glb_srst_ctrl, watchdog -> first global reset) já vem ligado, mas o bit 4
 * (pmu_srst_wdt_en) não, e o watchdog também reseta o PMU, o que desliga o aparelho
 * (só volta pelo botão). Com os bits 1 e 4 o watchdog faz o mesmo reset do reinício
 * normal. Medido no aparelho pelo kernel do Alpine: 0x103 -> 0x113 com escrita simples
 * (o registrador não tem máscara de escrita nos 16 bits altos).
 */
#include <linux/bits.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/printk.h>

#define RK3568_CRU_BASE		0xfdd20000
#define RK3568_GLB_RST_CON	0xdc
#define WDT_FIRST_GLB_SRST	BIT(1)	/* wdt_glb_srst_ctrl */
#define WDT_NO_PMU_SRST		BIT(4)	/* pmu_srst_wdt_en: o watchdog não reseta o PMU */
#define WDT_REBOOT_BITS		(WDT_FIRST_GLB_SRST | WDT_NO_PMU_SRST)

static int __init hy300_wdt_reboot_init(void)
{
	void __iomem *con;
	u32 before, after;

	if (!of_machine_is_compatible("rockchip,rk3566"))
		return 0;

	con = ioremap(RK3568_CRU_BASE + RK3568_GLB_RST_CON, 4);
	if (!con)
		return -ENOMEM;

	before = readl(con);
	writel(before | WDT_REBOOT_BITS, con);
	after = readl(con);
	iounmap(con);

	pr_info("hy300: watchdog com first global reset (CRU_GLB_RST_CON 0x%08x -> 0x%08x)%s\n",
		before, after, (after & WDT_REBOOT_BITS) == WDT_REBOOT_BITS ? "" : ": os bits não pegaram");
	return 0;
}
early_initcall(hy300_wdt_reboot_init);
