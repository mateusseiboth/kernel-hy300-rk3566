// SPDX-License-Identifier: GPL-2.0
/*
 * KUnit do motor de foco do HY300: comandos do sysfs, sequência de fases e fim de curso,
 * com GPIOs e espera falsos. Incluído no fim do hy300_focus_motor.c.
 */
#include <kunit/test.h>

#define FAKE_MAX_PHASES 8192

struct fake_hw {
	u8 phases[FAKE_MAX_PHASES][HY300_MOTOR_COILS];
	unsigned int nphases;
	unsigned int limit_after;	/* fim de curso aparece depois de N fases (0 = nunca) */
	unsigned int limit_until;	/* e some depois de M fases (0 = nunca some) */
	unsigned int delays;
};

static void fake_set(void *ctx, const u8 coils[HY300_MOTOR_COILS])
{
	struct fake_hw *hw = ctx;

	if (hw->nphases < FAKE_MAX_PHASES)
		memcpy(hw->phases[hw->nphases], coils, HY300_MOTOR_COILS);
	hw->nphases++;
}

static bool fake_at_limit(void *ctx)
{
	struct fake_hw *hw = ctx;

	if (!hw->limit_after || hw->nphases < hw->limit_after)
		return false;
	return !hw->limit_until || hw->nphases < hw->limit_until;
}

static void fake_delay(void *ctx)
{
	((struct fake_hw *)ctx)->delays++;
}

static const struct hy300_motor_ops fake_ops = {
	.set_coils = fake_set,
	.at_limit = fake_at_limit,
	.step_delay = fake_delay,
};

static struct hy300_motor *fake_motor(struct kunit *test, struct fake_hw **hw)
{
	struct hy300_motor *m = kunit_kzalloc(test, sizeof(*m), GFP_KERNEL);

	*hw = kunit_kzalloc(test, sizeof(**hw), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, m);
	KUNIT_ASSERT_NOT_NULL(test, *hw);
	hy300_motor_init(m, &fake_ops, *hw);
	return m;
}

/* Fases por ciclo: as 8 do meio passo e a final com as bobinas desligadas. */
#define PHASES_PER_CYCLE (HY300_MOTOR_HALF_STEPS + 1)

static void parse_accepts_factory_commands_with_or_without_newline(struct kunit *test)
{
	KUNIT_EXPECT_EQ(test, hy300_motor_parse("user_cw"), HY300_MOTOR_USER_CW);
	KUNIT_EXPECT_EQ(test, hy300_motor_parse("user_ccw\n"), HY300_MOTOR_USER_CCW);
	KUNIT_EXPECT_EQ(test, hy300_motor_parse("aacw"), HY300_MOTOR_AUTO_CW);
	KUNIT_EXPECT_EQ(test, hy300_motor_parse("aaccw"), HY300_MOTOR_AUTO_CCW);
	KUNIT_EXPECT_EQ(test, hy300_motor_parse("aacw_end"), HY300_MOTOR_END_CW);
	KUNIT_EXPECT_EQ(test, hy300_motor_parse("aaccw_end"), HY300_MOTOR_END_CCW);
	KUNIT_EXPECT_EQ(test, hy300_motor_parse("aastop"), HY300_MOTOR_STOP);
}

static void parse_rejects_unknown_commands(struct kunit *test)
{
	KUNIT_EXPECT_EQ(test, hy300_motor_parse("stop"), HY300_MOTOR_INVALID);
	KUNIT_EXPECT_EQ(test, hy300_motor_parse("cw"), HY300_MOTOR_INVALID);
	KUNIT_EXPECT_EQ(test, hy300_motor_parse("user_cw2"), HY300_MOTOR_INVALID);
	KUNIT_EXPECT_EQ(test, hy300_motor_parse(""), HY300_MOTOR_INVALID);
}

static void cw_cycle_is_factory_half_step_sequence(struct kunit *test)
{
	static const u8 want[PHASES_PER_CYCLE][HY300_MOTOR_COILS] = {
		{1, 0, 0, 0}, {1, 1, 0, 0}, {0, 1, 0, 0}, {0, 1, 1, 0},
		{0, 0, 1, 0}, {0, 0, 1, 1}, {0, 0, 0, 1}, {1, 0, 0, 1}, {0, 0, 0, 0},
	};
	struct fake_hw *hw;
	struct hy300_motor *m = fake_motor(test, &hw);

	hy300_motor_cycle(m, HY300_MOTOR_DIR_CW);
	KUNIT_ASSERT_EQ(test, hw->nphases, PHASES_PER_CYCLE);
	KUNIT_EXPECT_EQ(test, memcmp(hw->phases, want, sizeof(want)), 0);
	KUNIT_EXPECT_EQ(test, hw->delays, PHASES_PER_CYCLE);
}

static void ccw_cycle_is_cw_reversed(struct kunit *test)
{
	static const u8 want[PHASES_PER_CYCLE][HY300_MOTOR_COILS] = {
		{1, 0, 0, 1}, {0, 0, 0, 1}, {0, 0, 1, 1}, {0, 0, 1, 0},
		{0, 1, 1, 0}, {0, 1, 0, 0}, {1, 1, 0, 0}, {1, 0, 0, 0}, {0, 0, 0, 0},
	};
	struct fake_hw *hw;
	struct hy300_motor *m = fake_motor(test, &hw);

	hy300_motor_cycle(m, HY300_MOTOR_DIR_CCW);
	KUNIT_ASSERT_EQ(test, hw->nphases, PHASES_PER_CYCLE);
	KUNIT_EXPECT_EQ(test, memcmp(hw->phases, want, sizeof(want)), 0);
}

static void user_cw_runs_ten_cycles_even_at_limit(struct kunit *test)
{
	struct fake_hw *hw;
	struct hy300_motor *m = fake_motor(test, &hw);

	hw->limit_after = 1;
	KUNIT_EXPECT_EQ(test, hy300_motor_run(m, HY300_MOTOR_USER_CW), 0);
	KUNIT_EXPECT_EQ(test, hw->nphases, 10 * PHASES_PER_CYCLE);
	KUNIT_EXPECT_EQ(test, hw->phases[0][0], 1);
	KUNIT_EXPECT_EQ(test, hw->phases[0][3], 0);
}

static void user_ccw_starts_with_ccw_phase(struct kunit *test)
{
	struct fake_hw *hw;
	struct hy300_motor *m = fake_motor(test, &hw);

	KUNIT_EXPECT_EQ(test, hy300_motor_run(m, HY300_MOTOR_USER_CCW), 0);
	KUNIT_EXPECT_EQ(test, hw->nphases, 10 * PHASES_PER_CYCLE);
	KUNIT_EXPECT_EQ(test, hw->phases[0][3], 1);
}

static void auto_cw_moves_seven_cycles_without_limit(struct kunit *test)
{
	struct fake_hw *hw;
	struct hy300_motor *m = fake_motor(test, &hw);

	KUNIT_EXPECT_EQ(test, hy300_motor_run(m, HY300_MOTOR_AUTO_CW), 0);
	KUNIT_EXPECT_EQ(test, hw->nphases, 7 * PHASES_PER_CYCLE);
	KUNIT_EXPECT_EQ(test, m->end_hits, 0);
}

static void auto_cw_backs_off_when_limit_is_hit(struct kunit *test)
{
	struct fake_hw *hw;
	struct hy300_motor *m = fake_motor(test, &hw);

	/* fim de curso depois de 3 ciclos; some depois de 3 + 10 ciclos de recuo */
	hw->limit_after = 3 * PHASES_PER_CYCLE;
	hw->limit_until = 13 * PHASES_PER_CYCLE;
	KUNIT_EXPECT_EQ(test, hy300_motor_run(m, HY300_MOTOR_AUTO_CW), 0);
	KUNIT_EXPECT_EQ(test, m->end_hits, 1);
	KUNIT_EXPECT_EQ(test, hw->nphases, 13 * PHASES_PER_CYCLE);
	/* o primeiro ciclo de recuo é anti-horário */
	KUNIT_EXPECT_EQ(test, hw->phases[3 * PHASES_PER_CYCLE][3], 1);
}

static void auto_backoff_gives_up_after_three_rounds(struct kunit *test)
{
	struct fake_hw *hw;
	struct hy300_motor *m = fake_motor(test, &hw);

	hw->limit_after = 1;	/* fim de curso ativo depois da primeira fase e preso */
	KUNIT_EXPECT_EQ(test, hy300_motor_run(m, HY300_MOTOR_AUTO_CCW), 0);
	/* um ciclo para a frente (o fim de curso ainda não estava ativo) e 3 rodadas de recuo */
	KUNIT_EXPECT_EQ(test, hw->nphases, (1 + 3 * 10) * PHASES_PER_CYCLE);
}

static void end_ccw_runs_until_limit_then_backs_off(struct kunit *test)
{
	struct fake_hw *hw;
	struct hy300_motor *m = fake_motor(test, &hw);

	hw->limit_after = 50 * PHASES_PER_CYCLE;
	hw->limit_until = 60 * PHASES_PER_CYCLE;
	KUNIT_EXPECT_EQ(test, hy300_motor_run(m, HY300_MOTOR_END_CCW), 0);
	KUNIT_EXPECT_EQ(test, hw->nphases, 60 * PHASES_PER_CYCLE);
	KUNIT_EXPECT_EQ(test, m->end_hits, 1);
}

static void end_cw_stops_at_max_cycles_without_limit(struct kunit *test)
{
	struct fake_hw *hw;
	struct hy300_motor *m = fake_motor(test, &hw);

	KUNIT_EXPECT_EQ(test, hy300_motor_run(m, HY300_MOTOR_END_CW), 0);
	KUNIT_EXPECT_EQ(test, hw->nphases, HY300_MOTOR_END_MAX_CYCLES * PHASES_PER_CYCLE);
}

static void stop_interrupts_a_running_move(struct kunit *test)
{
	struct fake_hw *hw;
	struct hy300_motor *m = fake_motor(test, &hw);

	atomic_set(&m->stop, 1);
	KUNIT_EXPECT_EQ(test, hy300_motor_steps(m, HY300_MOTOR_DIR_CW, 10, false), 0);
	KUNIT_EXPECT_EQ(test, hw->nphases, 0);
}

static void stop_turns_coils_off_and_resets_end_hits(struct kunit *test)
{
	static const u8 off[HY300_MOTOR_COILS];
	struct fake_hw *hw;
	struct hy300_motor *m = fake_motor(test, &hw);

	m->end_hits = 5;
	KUNIT_EXPECT_EQ(test, hy300_motor_run(m, HY300_MOTOR_STOP), 0);
	KUNIT_EXPECT_EQ(test, m->end_hits, 0);
	KUNIT_ASSERT_EQ(test, hw->nphases, 1);
	KUNIT_EXPECT_EQ(test, memcmp(hw->phases[0], off, sizeof(off)), 0);
}

static void invalid_command_does_not_move(struct kunit *test)
{
	struct fake_hw *hw;
	struct hy300_motor *m = fake_motor(test, &hw);

	KUNIT_EXPECT_EQ(test, hy300_motor_run(m, HY300_MOTOR_INVALID), -EINVAL);
	KUNIT_EXPECT_EQ(test, hw->nphases, 0);
}

static void ir_burst_moves_four_cycles(struct kunit *test)
{
	struct fake_hw *hw;
	struct hy300_motor *m = fake_motor(test, &hw);

	KUNIT_EXPECT_TRUE(test, hy300_motor_ir_burst(m, HY300_MOTOR_DIR_CW));
	KUNIT_EXPECT_EQ(test, hw->nphases, 4 * PHASES_PER_CYCLE);
}

static void ir_burst_at_limit_backs_off_twenty_and_ends_hold(struct kunit *test)
{
	struct fake_hw *hw;
	struct hy300_motor *m = fake_motor(test, &hw);

	hw->limit_after = 2 * PHASES_PER_CYCLE;
	KUNIT_EXPECT_FALSE(test, hy300_motor_ir_burst(m, HY300_MOTOR_DIR_CW));
	KUNIT_EXPECT_EQ(test, hw->nphases, (2 + 20) * PHASES_PER_CYCLE);
}

static void ir_key_maps_f15_to_cw_and_f16_to_ccw(struct kunit *test)
{
	KUNIT_EXPECT_EQ(test, hy300_motor_ir_dir(KEY_F15), HY300_MOTOR_DIR_CW);
	KUNIT_EXPECT_EQ(test, hy300_motor_ir_dir(KEY_F16), HY300_MOTOR_DIR_CCW);
	KUNIT_EXPECT_EQ(test, hy300_motor_ir_dir(KEY_F14), HY300_MOTOR_DIR_NONE);
}

static struct kunit_case hy300_motor_cases[] = {
	KUNIT_CASE(parse_accepts_factory_commands_with_or_without_newline),
	KUNIT_CASE(parse_rejects_unknown_commands),
	KUNIT_CASE(cw_cycle_is_factory_half_step_sequence),
	KUNIT_CASE(ccw_cycle_is_cw_reversed),
	KUNIT_CASE(user_cw_runs_ten_cycles_even_at_limit),
	KUNIT_CASE(user_ccw_starts_with_ccw_phase),
	KUNIT_CASE(auto_cw_moves_seven_cycles_without_limit),
	KUNIT_CASE(auto_cw_backs_off_when_limit_is_hit),
	KUNIT_CASE(auto_backoff_gives_up_after_three_rounds),
	KUNIT_CASE(end_ccw_runs_until_limit_then_backs_off),
	KUNIT_CASE(end_cw_stops_at_max_cycles_without_limit),
	KUNIT_CASE(stop_interrupts_a_running_move),
	KUNIT_CASE(stop_turns_coils_off_and_resets_end_hits),
	KUNIT_CASE(invalid_command_does_not_move),
	KUNIT_CASE(ir_burst_moves_four_cycles),
	KUNIT_CASE(ir_burst_at_limit_backs_off_twenty_and_ends_hold),
	KUNIT_CASE(ir_key_maps_f15_to_cw_and_f16_to_ccw),
	{}
};

static struct kunit_suite hy300_motor_suite = {
	.name = "hy300_focus_motor",
	.test_cases = hy300_motor_cases,
};
kunit_test_suite(hy300_motor_suite);
