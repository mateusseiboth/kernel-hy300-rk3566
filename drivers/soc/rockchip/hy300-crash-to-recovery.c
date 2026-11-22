// SPDX-License-Identifier: GPL-2.0
/*
 * HY300: travamento e panic voltam pela partição recovery.
 *
 * Com o Alpine de recuperação na partição boot e o Android na recovery, um kernel que dá panic
 * ou trava (e o watchdog reseta) voltaria para a boot: o mesmo kernel, em laço, e tirar da
 * tomada não ajuda. Com hy300.crash_to_recovery=1 na cmdline (só a imagem da boot leva):
 *  - no early_initcall o registrador do reboot-mode (PMU_GRF + 0x200) recebe BOOT_RECOVERY.
 *    Um reset do watchdog ou do TSADC não escreve nada, então o U-Boot sobe a recovery;
 *  - um notificador de panic, depois do do reboot-mode (que grava "panic"), grava
 *    BOOT_RECOVERY de novo.
 * Reboots pedidos continuam como antes: o reboot-mode grava normal ou o modo pedido.
 */
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/limits.h>
#include <linux/notifier.h>
#include <linux/panic_notifier.h>
#include <linux/printk.h>
#include <dt-bindings/soc/rockchip,boot-mode.h>

#define HY300_REBOOT_MODE_REG	0xfdc20200	/* PMU_GRF, offset do syscon-reboot-mode */

static bool hy300_crash_to_recovery;
static void __iomem *hy300_reboot_mode;

static int __init hy300_parse_crash_to_recovery(char *arg)
{
	hy300_crash_to_recovery = arg && !strcmp(arg, "1");
	return 0;
}
early_param("hy300.crash_to_recovery", hy300_parse_crash_to_recovery);

static void hy300_write_recovery_mode(void)
{
	writel(BOOT_RECOVERY, hy300_reboot_mode);
}

static int hy300_panic_to_recovery(struct notifier_block *nb, unsigned long event, void *data)
{
	hy300_write_recovery_mode();
	return NOTIFY_DONE;
}

static struct notifier_block hy300_panic_nb = {
	.notifier_call = hy300_panic_to_recovery,
	.priority = INT_MIN,
};

static int __init hy300_crash_to_recovery_init(void)
{
	if (!hy300_crash_to_recovery)
		return 0;
	hy300_reboot_mode = ioremap(HY300_REBOOT_MODE_REG, sizeof(u32));
	if (!hy300_reboot_mode)
		return -ENOMEM;
	hy300_write_recovery_mode();
	atomic_notifier_chain_register(&panic_notifier_list, &hy300_panic_nb);
	pr_info("hy300: panic e travamento voltam pela partição recovery\n");
	return 0;
}
early_initcall(hy300_crash_to_recovery_init);
