# SPDX-License-Identifier: GPL-2.0
# HY300: só o rfkill do Bluetooth (aic_bluetooth_main + rfkill). O lpm.c (bluesleep) do SDK não
# entra: com CONFIG_PLATFORM_ROCKCHIP2 o aic_bluetooth_main não o chama, e o wake do BT fica com
# o rfkill-rk (bluetooth-platdata) do kernel da Rockchip, como na fábrica.
obj-$(CONFIG_AIC8800_BTLPM) += aic8800_btlpm.o
aic8800_btlpm-y := aic_bluetooth_main.o rfkill.o
ccflags-y += -DANDROID_PLATFORM -DCONFIG_PLATFORM_ROCKCHIP2
