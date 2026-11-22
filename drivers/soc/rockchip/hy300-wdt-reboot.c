// SPDX-License-Identifier: GPL-2.0
/*
 * HY300 (RK3566): o reset do watchdog do SoC vira reinício, não desligamento.
 *
 * O dw-wdt aciona o reset global escolhido no CRU_GLB_RST_CON. No valor de reset
 * (bit 1 = 0) é o "second global reset", que no HY300 desliga o aparelho: só volta
 * pelo botão. O reinício por PSCI, panic ou sysrq b usa o "first global reset"
 * (CRU_GLB_SRST_FST) e volta normalmente. Com o bit 1 em 1 o watchdog usa o mesmo
 * reset do reinício normal.
 *
 * Layout do registrador como no RK3368/PX30 (U-Boot, cru_rk3368.h): bit 0 TSADC e
 * bit 1 WDT, 0 = second e 1 = first global reset. O TSADC (desligamento por
 * temperatura) fica como está. O registrador pode ou não ter a máscara de escrita
 * nos 16 bits altos; a escrita simples vale no primeiro caso e é ignorada no
 * segundo, e a com máscara só é tentada se a simples não pegou.
 */
#include <linux/bits.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/printk.h>

#define RK3568_CRU_BASE		0xfdd20000
#define RK3568_GLB_RST_CON	0xdc
#define WDT_FIRST_GLB_SRST	BIT(1)

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
	writel(before | WDT_FIRST_GLB_SRST, con);
	if (!(readl(con) & WDT_FIRST_GLB_SRST))
		writel((WDT_FIRST_GLB_SRST << 16) | WDT_FIRST_GLB_SRST, con);
	after = readl(con);
	iounmap(con);

	pr_info("hy300: watchdog com first global reset (CRU_GLB_RST_CON 0x%08x -> 0x%08x)%s\n",
		before, after, after & WDT_FIRST_GLB_SRST ? "" : ": o bit não pegou");
	return 0;
}
early_initcall(hy300_wdt_reboot_init);
