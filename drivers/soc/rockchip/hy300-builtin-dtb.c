// SPDX-License-Identifier: GPL-2.0
/*
 * DT embutido no kernel para o HY300.
 *
 * O U-Boot de fábrica (TXCZ, 2017.09) passa a usar o rk-kernel.dtb do resource.img como DT
 * próprio antes de ler a imagem e não funciona com o DT do 6.1: trava antes do logo. Por
 * isso o resource leva o DT de fábrica (o U-Boot segue como sempre) e o kernel usa este,
 * copiando do DT que o U-Boot entregou só o que ele preenche no boot:
 *  - todas as propriedades do /chosen (cmdline, initrd, sementes);
 *  - o reg dos nós de memória, já sem as áreas do TF-A e do OP-TEE;
 *  - as entradas /memreserve/.
 */
#include <linux/init.h>
#include <linux/libfdt.h>
#include <linux/printk.h>
#include <linux/sizes.h>
#include <linux/string.h>

extern u8 __dtb_hy300_builtin_begin[];

/* Fora do __init: o kernel continua apontando para o DT achatado depois do boot. */
static u8 hy300_fdt[SZ_512K] __aligned(8);

static int __init hy300_copy_props(const void *src, int src_node, void *dst, int dst_node)
{
	int prop, len, err;
	const char *name;
	const void *val;

	fdt_for_each_property_offset(prop, src, src_node) {
		val = fdt_getprop_by_offset(src, prop, &name, &len);
		if (!val)
			return -FDT_ERR_BADSTRUCTURE;
		err = fdt_setprop(dst, dst_node, name, val, len);
		if (err)
			return err;
	}
	return 0;
}

static int __init hy300_require_node(void *fdt, const char *path, const char *name)
{
	int node = fdt_path_offset(fdt, path);

	return node >= 0 ? node : fdt_add_subnode(fdt, 0, name);
}

static int __init hy300_copy_chosen(const void *boot, void *fdt)
{
	int src = fdt_path_offset(boot, "/chosen");
	int dst = hy300_require_node(fdt, "/chosen", "chosen");

	if (src < 0)
		return 0;
	return dst < 0 ? dst : hy300_copy_props(boot, src, fdt, dst);
}

static bool __init hy300_is_memory(const void *fdt, int node)
{
	const char *type = fdt_getprop(fdt, node, "device_type", NULL);

	return type && !strcmp(type, "memory");
}

/* Apaga os nós de memória do DT embutido; o U-Boot é quem sabe o tamanho real. */
static void __init hy300_drop_memory(void *fdt)
{
	int node = fdt_first_subnode(fdt, 0);

	while (node >= 0) {
		if (hy300_is_memory(fdt, node)) {
			fdt_del_node(fdt, node);
			node = fdt_first_subnode(fdt, 0);
			continue;
		}
		node = fdt_next_subnode(fdt, node);
	}
}

static int __init hy300_copy_memory(const void *boot, void *fdt)
{
	int src, dst, err;

	hy300_drop_memory(fdt);
	fdt_for_each_subnode(src, boot, 0) {
		if (!hy300_is_memory(boot, src))
			continue;
		dst = fdt_add_subnode(fdt, 0, fdt_get_name(boot, src, NULL));
		if (dst < 0)
			return dst;
		err = hy300_copy_props(boot, src, fdt, dst);
		if (err)
			return err;
	}
	return 0;
}

static int __init hy300_copy_memreserve(const void *boot, void *fdt)
{
	int i, err, n = fdt_num_mem_rsv(boot);
	u64 addr, size;

	for (i = 0; i < n; i++) {
		err = fdt_get_mem_rsv(boot, i, &addr, &size);
		if (!err)
			err = fdt_add_mem_rsv(fdt, addr, size);
		if (err)
			return err;
	}
	return 0;
}

/* Devolve o DT a usar: o embutido com os dados do U-Boot, ou o do U-Boot se algo falhar. */
void * __init hy300_builtin_fdt(void *boot)
{
	int err = fdt_check_header(boot);

	if (!err)
		err = fdt_open_into(__dtb_hy300_builtin_begin, hy300_fdt, sizeof(hy300_fdt));
	if (!err)
		err = hy300_copy_chosen(boot, hy300_fdt);
	if (!err)
		err = hy300_copy_memory(boot, hy300_fdt);
	if (!err)
		err = hy300_copy_memreserve(boot, hy300_fdt);
	if (!err)
		err = fdt_pack(hy300_fdt);
	if (err) {
		pr_err("hy300: DT embutido falhou (%d), usando o do bootloader\n", err);
		return boot;
	}
	pr_info("hy300: usando o DT embutido\n");
	return hy300_fdt;
}
