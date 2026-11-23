// SPDX-License-Identifier: GPL-2.0
/*
 * Motor de foco do projetor HY300 (TXCZ).
 *
 * Driver novo, com a interface do "usb-cam-gpio" (rk_motor) do kernel 4.19 de fábrica, que não
 * tem fonte publicada. O comportamento foi inferido da desmontagem do Image de fábrica
 * (docs/kernel-6.1.md do hy300-rk3566, "Motor de foco"):
 *
 *  - motor de passo de 4 fios nos GPIO4_C2..C5 (gpio 146..149, fixos no código de fábrica),
 *    meio passo de 8 fases, 1,5 ms por fase, bobinas desligadas no fim de cada ciclo;
 *  - fim de curso no GPIO4_D1 (gpio 153), 1 = no limite;
 *  - /sys/class/motor_cc/motor_cc/motor: user_cw e user_ccw andam 10 ciclos sem olhar o fim
 *    de curso; aacw e aaccw andam 7 e, no fim de curso, recuam 10 por vez (até 3 vezes);
 *    aacw_end e aaccw_end andam até o fim de curso e recuam; aastop desliga as bobinas;
 *  - teclas F15 (horário) e F16 (anti-horário) do controle: 4 ciclos ao apertar e mais 4 a
 *    cada intervalo enquanto seguradas; no fim de curso, recua 20 e para.
 *
 * Diferenças de propósito: a escrita devolve o tamanho (a de fábrica devolvia -EINVAL para
 * qualquer texto, porque tentava kstrtoint depois de tratar o comando), aceita "\n" no fim
 * (a de fábrica usava strcmp) e aastop interrompe um movimento em andamento.
 */
#include <linux/atomic.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/input.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/string.h>
#include <linux/workqueue.h>

#define HY300_MOTOR_COILS		4
#define HY300_MOTOR_HALF_STEPS		8
#define HY300_MOTOR_PHASE_US		1500
#define HY300_MOTOR_USER_CYCLES		10
#define HY300_MOTOR_AUTO_CYCLES		7
#define HY300_MOTOR_BACKOFF_CYCLES	10
#define HY300_MOTOR_BACKOFF_ROUNDS	3
#define HY300_MOTOR_END_MAX_CYCLES	400
#define HY300_MOTOR_IR_CYCLES		4
#define HY300_MOTOR_IR_BACKOFF_CYCLES	20
#define HY300_MOTOR_IR_FIRST_DELAY_MS	550
#define HY300_MOTOR_IR_REPEAT_MS	150

enum hy300_motor_dir {
	HY300_MOTOR_DIR_NONE,
	HY300_MOTOR_DIR_CW,
	HY300_MOTOR_DIR_CCW,
};

enum hy300_motor_cmd {
	HY300_MOTOR_INVALID,
	HY300_MOTOR_USER_CW,
	HY300_MOTOR_USER_CCW,
	HY300_MOTOR_AUTO_CW,
	HY300_MOTOR_AUTO_CCW,
	HY300_MOTOR_END_CW,
	HY300_MOTOR_END_CCW,
	HY300_MOTOR_STOP,
};

struct hy300_motor_ops {
	void (*set_coils)(void *ctx, const u8 coils[HY300_MOTOR_COILS]);
	bool (*at_limit)(void *ctx);
	void (*step_delay)(void *ctx);
};

struct hy300_motor {
	const struct hy300_motor_ops *ops;
	void *ctx;
	struct mutex lock;	/* um movimento por vez */
	atomic_t stop;
	int end_hits;		/* "v_out" de fábrica: quantas vezes o fim de curso foi atingido */
};

struct hy300_motor_dev {
	struct hy300_motor motor;
	struct gpio_descs *phases;
	struct gpio_desc *limit;
	struct gpio_desc *hd_cam;
	struct device *sysfs_dev;
	struct input_handler ir;
	struct delayed_work ir_work;
	atomic_t ir_dir;
};

/* Fases do sentido horário, na ordem do motor_cw de fábrica; o anti-horário é a ordem inversa. */
static const u8 hy300_motor_cw[HY300_MOTOR_HALF_STEPS][HY300_MOTOR_COILS] = {
	{1, 0, 0, 0}, {1, 1, 0, 0}, {0, 1, 0, 0}, {0, 1, 1, 0},
	{0, 0, 1, 0}, {0, 0, 1, 1}, {0, 0, 0, 1}, {1, 0, 0, 1},
};

static const u8 hy300_motor_off[HY300_MOTOR_COILS];

static const char *const hy300_motor_cmd_names[] = {
	[HY300_MOTOR_USER_CW] = "user_cw",
	[HY300_MOTOR_USER_CCW] = "user_ccw",
	[HY300_MOTOR_AUTO_CW] = "aacw",
	[HY300_MOTOR_AUTO_CCW] = "aaccw",
	[HY300_MOTOR_END_CW] = "aacw_end",
	[HY300_MOTOR_END_CCW] = "aaccw_end",
	[HY300_MOTOR_STOP] = "aastop",
};

static void hy300_motor_init(struct hy300_motor *m, const struct hy300_motor_ops *ops, void *ctx)
{
	m->ops = ops;
	m->ctx = ctx;
	mutex_init(&m->lock);
	atomic_set(&m->stop, 0);
	m->end_hits = 0;
}

static enum hy300_motor_cmd hy300_motor_parse(const char *buf)
{
	int i;

	for (i = HY300_MOTOR_USER_CW; i < ARRAY_SIZE(hy300_motor_cmd_names); i++)
		if (sysfs_streq(buf, hy300_motor_cmd_names[i]))
			return i;
	return HY300_MOTOR_INVALID;
}

static enum hy300_motor_dir hy300_motor_reverse(enum hy300_motor_dir dir)
{
	return dir == HY300_MOTOR_DIR_CW ? HY300_MOTOR_DIR_CCW : HY300_MOTOR_DIR_CW;
}

static void hy300_motor_set(struct hy300_motor *m, const u8 coils[HY300_MOTOR_COILS])
{
	m->ops->set_coils(m->ctx, coils);
	m->ops->step_delay(m->ctx);
}

static void hy300_motor_cycle(struct hy300_motor *m, enum hy300_motor_dir dir)
{
	int i;

	for (i = 0; i < HY300_MOTOR_HALF_STEPS; i++)
		hy300_motor_set(m, hy300_motor_cw[dir == HY300_MOTOR_DIR_CW ? i : HY300_MOTOR_HALF_STEPS - 1 - i]);
	hy300_motor_set(m, hy300_motor_off);
}

/* Anda até n ciclos; com stop_at_limit, para antes de um ciclo se o fim de curso estiver ativo. */
static int hy300_motor_steps(struct hy300_motor *m, enum hy300_motor_dir dir, int n, bool stop_at_limit)
{
	int done;

	for (done = 0; done < n; done++) {
		if (atomic_read(&m->stop))
			break;
		if (stop_at_limit && m->ops->at_limit(m->ctx))
			break;
		hy300_motor_cycle(m, dir);
	}
	return done;
}

/* No fim de curso: recua em rodadas de 10 ciclos enquanto ele continuar ativo. */
static void hy300_motor_back_off(struct hy300_motor *m, enum hy300_motor_dir dir)
{
	int round;

	if (!m->ops->at_limit(m->ctx))
		return;
	m->end_hits++;
	for (round = 0; round < HY300_MOTOR_BACKOFF_ROUNDS && m->ops->at_limit(m->ctx); round++)
		hy300_motor_steps(m, hy300_motor_reverse(dir), HY300_MOTOR_BACKOFF_CYCLES, false);
}

static void hy300_motor_move(struct hy300_motor *m, enum hy300_motor_dir dir, int cycles)
{
	hy300_motor_steps(m, dir, cycles, true);
	hy300_motor_back_off(m, dir);
}

static void hy300_motor_stop(struct hy300_motor *m)
{
	atomic_set(&m->stop, 1);
	m->ops->set_coils(m->ctx, hy300_motor_off);
	m->end_hits = 0;
}

static int hy300_motor_run(struct hy300_motor *m, enum hy300_motor_cmd cmd)
{
	if (cmd == HY300_MOTOR_INVALID)
		return -EINVAL;
	if (cmd == HY300_MOTOR_STOP) {
		hy300_motor_stop(m);
		return 0;
	}

	mutex_lock(&m->lock);
	atomic_set(&m->stop, 0);
	switch (cmd) {
	case HY300_MOTOR_USER_CW:
	case HY300_MOTOR_USER_CCW:
		hy300_motor_steps(m, cmd == HY300_MOTOR_USER_CW ? HY300_MOTOR_DIR_CW : HY300_MOTOR_DIR_CCW,
				  HY300_MOTOR_USER_CYCLES, false);
		break;
	case HY300_MOTOR_AUTO_CW:
	case HY300_MOTOR_AUTO_CCW:
		hy300_motor_move(m, cmd == HY300_MOTOR_AUTO_CW ? HY300_MOTOR_DIR_CW : HY300_MOTOR_DIR_CCW,
				 HY300_MOTOR_AUTO_CYCLES);
		break;
	default:
		hy300_motor_move(m, cmd == HY300_MOTOR_END_CW ? HY300_MOTOR_DIR_CW : HY300_MOTOR_DIR_CCW,
				 HY300_MOTOR_END_MAX_CYCLES);
		break;
	}
	mutex_unlock(&m->lock);
	return 0;
}

static enum hy300_motor_dir hy300_motor_ir_dir(unsigned int code)
{
	if (code == KEY_F15)
		return HY300_MOTOR_DIR_CW;
	if (code == KEY_F16)
		return HY300_MOTOR_DIR_CCW;
	return HY300_MOTOR_DIR_NONE;
}

/* Um surto do controle remoto. Devolve false quando bateu no fim de curso (fim do segurar). */
static bool hy300_motor_ir_burst(struct hy300_motor *m, enum hy300_motor_dir dir)
{
	int done = hy300_motor_steps(m, dir, HY300_MOTOR_IR_CYCLES, true);

	if (done == HY300_MOTOR_IR_CYCLES && !m->ops->at_limit(m->ctx))
		return true;
	if (m->ops->at_limit(m->ctx)) {
		m->end_hits++;
		hy300_motor_steps(m, hy300_motor_reverse(dir), HY300_MOTOR_IR_BACKOFF_CYCLES, false);
	}
	return false;
}

/* GPIOs de verdade */

static void hy300_motor_gpio_set(void *ctx, const u8 coils[HY300_MOTOR_COILS])
{
	struct hy300_motor_dev *md = ctx;
	DECLARE_BITMAP(values, HY300_MOTOR_COILS) = { 0 };
	int i;

	for (i = 0; i < HY300_MOTOR_COILS; i++)
		__assign_bit(i, values, coils[i]);
	gpiod_set_array_value_cansleep(md->phases->ndescs, md->phases->desc, md->phases->info, values);
}

static bool hy300_motor_gpio_at_limit(void *ctx)
{
	struct hy300_motor_dev *md = ctx;

	return gpiod_get_value_cansleep(md->limit) > 0;
}

static void hy300_motor_sleep(void *ctx)
{
	usleep_range(HY300_MOTOR_PHASE_US, HY300_MOTOR_PHASE_US + 100);
}

static const struct hy300_motor_ops hy300_motor_gpio_ops = {
	.set_coils = hy300_motor_gpio_set,
	.at_limit = hy300_motor_gpio_at_limit,
	.step_delay = hy300_motor_sleep,
};

/* sysfs: /sys/class/motor_cc/motor_cc/{motor,hd_camera_on} */

static ssize_t limit_show(struct device *dev, char *buf)
{
	struct hy300_motor_dev *md = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", hy300_motor_gpio_at_limit(md));
}

static ssize_t motor_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return limit_show(dev, buf);
}

static ssize_t motor_store(struct device *dev, struct device_attribute *attr,
			   const char *buf, size_t count)
{
	struct hy300_motor_dev *md = dev_get_drvdata(dev);
	int ret = hy300_motor_run(&md->motor, hy300_motor_parse(buf));

	return ret ? ret : count;
}
static DEVICE_ATTR_RW(motor);

static ssize_t hd_camera_on_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return limit_show(dev, buf);
}

/* Liga a câmera "HD" opcional (hd-cam-gpios); no HY300 de fábrica o DT não declara nenhuma. */
static ssize_t hd_camera_on_store(struct device *dev, struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct hy300_motor_dev *md = dev_get_drvdata(dev);
	int on;
	int ret = kstrtoint(buf, 0, &on);

	if (ret)
		return ret;
	gpiod_set_value_cansleep(md->hd_cam, !!on);
	return count;
}
static DEVICE_ATTR_RW(hd_camera_on);

static struct attribute *hy300_motor_attrs[] = {
	&dev_attr_motor.attr,
	&dev_attr_hd_camera_on.attr,
	NULL
};
ATTRIBUTE_GROUPS(hy300_motor);

static struct class *hy300_motor_class;

/* Controle remoto: F15/F16 seguradas movem o foco, como o motor_run de fábrica. */

static void hy300_motor_ir_work(struct work_struct *work)
{
	struct hy300_motor_dev *md = container_of(to_delayed_work(work), struct hy300_motor_dev, ir_work);
	enum hy300_motor_dir dir = atomic_read(&md->ir_dir);
	bool again;

	if (dir == HY300_MOTOR_DIR_NONE)
		return;
	mutex_lock(&md->motor.lock);
	atomic_set(&md->motor.stop, 0);
	again = hy300_motor_ir_burst(&md->motor, dir);
	mutex_unlock(&md->motor.lock);
	if (!again) {
		atomic_cmpxchg(&md->ir_dir, dir, HY300_MOTOR_DIR_NONE);
		return;
	}
	if (atomic_read(&md->ir_dir) == dir)
		schedule_delayed_work(&md->ir_work, msecs_to_jiffies(HY300_MOTOR_IR_REPEAT_MS));
}

static void hy300_motor_ir_event(struct input_handle *handle, unsigned int type,
				 unsigned int code, int value)
{
	struct hy300_motor_dev *md = handle->handler->private;
	enum hy300_motor_dir dir = hy300_motor_ir_dir(code);

	if (type != EV_KEY || dir == HY300_MOTOR_DIR_NONE || value == 2)
		return;
	if (!value) {
		atomic_cmpxchg(&md->ir_dir, dir, HY300_MOTOR_DIR_NONE);
		return;
	}
	atomic_set(&md->ir_dir, dir);
	mod_delayed_work(system_wq, &md->ir_work, 0);
	/* o segundo surto só depois de segurar, como o temporizador de fábrica */
	schedule_delayed_work(&md->ir_work, msecs_to_jiffies(HY300_MOTOR_IR_FIRST_DELAY_MS));
}

static int hy300_motor_ir_connect(struct input_handler *handler, struct input_dev *dev,
				  const struct input_device_id *id)
{
	struct input_handle *handle = kzalloc(sizeof(*handle), GFP_KERNEL);
	int ret;

	if (!handle)
		return -ENOMEM;
	handle->dev = dev;
	handle->handler = handler;
	handle->name = "hy300_focus_motor";
	ret = input_register_handle(handle);
	if (ret)
		goto err_free;
	ret = input_open_device(handle);
	if (ret)
		goto err_unregister;
	return 0;

err_unregister:
	input_unregister_handle(handle);
err_free:
	kfree(handle);
	return ret;
}

static void hy300_motor_ir_disconnect(struct input_handle *handle)
{
	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);
}

static const struct input_device_id hy300_motor_ir_ids[] = {
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT | INPUT_DEVICE_ID_MATCH_KEYBIT,
		.evbit = { BIT_MASK(EV_KEY) },
		.keybit = { [BIT_WORD(KEY_F15)] = BIT_MASK(KEY_F15) },
	},
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT | INPUT_DEVICE_ID_MATCH_KEYBIT,
		.evbit = { BIT_MASK(EV_KEY) },
		.keybit = { [BIT_WORD(KEY_F16)] = BIT_MASK(KEY_F16) },
	},
	{ }
};

static int hy300_motor_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct hy300_motor_dev *md;
	int ret;

	md = devm_kzalloc(dev, sizeof(*md), GFP_KERNEL);
	if (!md)
		return -ENOMEM;

	md->phases = devm_gpiod_get_array(dev, "phase", GPIOD_OUT_LOW);
	if (IS_ERR(md->phases))
		return dev_err_probe(dev, PTR_ERR(md->phases), "sem phase-gpios\n");
	if (md->phases->ndescs != HY300_MOTOR_COILS)
		return dev_err_probe(dev, -EINVAL, "phase-gpios precisa de %d GPIOs\n", HY300_MOTOR_COILS);
	md->limit = devm_gpiod_get(dev, "limit", GPIOD_IN);
	if (IS_ERR(md->limit))
		return dev_err_probe(dev, PTR_ERR(md->limit), "sem limit-gpios\n");
	md->hd_cam = devm_gpiod_get_optional(dev, "hd-cam", GPIOD_OUT_LOW);
	if (IS_ERR(md->hd_cam))
		return dev_err_probe(dev, PTR_ERR(md->hd_cam), "hd-cam-gpios inválido\n");

	hy300_motor_init(&md->motor, &hy300_motor_gpio_ops, md);
	INIT_DELAYED_WORK(&md->ir_work, hy300_motor_ir_work);
	atomic_set(&md->ir_dir, HY300_MOTOR_DIR_NONE);
	platform_set_drvdata(pdev, md);

	md->sysfs_dev = device_create_with_groups(hy300_motor_class, dev, MKDEV(0, 0), md,
						  hy300_motor_groups, "motor_cc");
	if (IS_ERR(md->sysfs_dev))
		return dev_err_probe(dev, PTR_ERR(md->sysfs_dev), "falha ao criar motor_cc\n");

	md->ir.event = hy300_motor_ir_event;
	md->ir.connect = hy300_motor_ir_connect;
	md->ir.disconnect = hy300_motor_ir_disconnect;
	md->ir.name = "hy300_focus_motor";
	md->ir.id_table = hy300_motor_ir_ids;
	md->ir.private = md;
	ret = input_register_handler(&md->ir);
	if (ret) {
		device_unregister(md->sysfs_dev);
		return dev_err_probe(dev, ret, "falha ao registrar o controle remoto\n");
	}
	return 0;
}

static int hy300_motor_remove(struct platform_device *pdev)
{
	struct hy300_motor_dev *md = platform_get_drvdata(pdev);

	input_unregister_handler(&md->ir);
	atomic_set(&md->ir_dir, HY300_MOTOR_DIR_NONE);
	cancel_delayed_work_sync(&md->ir_work);
	device_unregister(md->sysfs_dev);
	hy300_motor_stop(&md->motor);
	return 0;
}

static const struct of_device_id hy300_motor_of_match[] = {
	{ .compatible = "txcz,hy300-focus-motor" },
	{ }
};
MODULE_DEVICE_TABLE(of, hy300_motor_of_match);

static struct platform_driver hy300_motor_driver = {
	.probe = hy300_motor_probe,
	.remove = hy300_motor_remove,
	.driver = {
		.name = "hy300-focus-motor",
		.of_match_table = hy300_motor_of_match,
	},
};

static int __init hy300_motor_mod_init(void)
{
	int ret;

	hy300_motor_class = class_create(THIS_MODULE, "motor_cc");
	if (IS_ERR(hy300_motor_class))
		return PTR_ERR(hy300_motor_class);
	ret = platform_driver_register(&hy300_motor_driver);
	if (ret)
		class_destroy(hy300_motor_class);
	return ret;
}
module_init(hy300_motor_mod_init);

static void __exit hy300_motor_mod_exit(void)
{
	platform_driver_unregister(&hy300_motor_driver);
	class_destroy(hy300_motor_class);
}
module_exit(hy300_motor_mod_exit);

#if IS_ENABLED(CONFIG_HY300_FOCUS_MOTOR_KUNIT_TEST)
#include "hy300_focus_motor_test.c"
#endif

MODULE_DESCRIPTION("Motor de foco do projetor HY300 (motor_cc)");
MODULE_LICENSE("GPL");
