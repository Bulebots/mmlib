#include "hmi.h"

/**
 * @brief Blink LEDs a defined number of times.
 *
 * @param[in] count number of blinks.
 * @param[in] time that LEDs are ON and OFF.
 */
void repeat_blink(uint8_t count, uint16_t time)
{
	int i;

	for (i = 0; i < count; i++) {
		led_left_on();
		led_right_on();
		sleep_ticks(time);
		led_left_off();
		led_right_off();
		sleep_ticks(time);
	}
}

/**
 * @brief Blink both LEDs alternately to report collision detection.
 */
void blink_collision(void)
{
	int i;

	for (i = 0; i < 10; i++) {
		led_left_on();
		led_right_off();
		sleep_ticks(200);
		led_left_off();
		led_right_on();
		sleep_ticks(200);
	}
	led_right_off();
}

/**
 * @brief Read user button, requiring consecutive positive reads.
 *
 * @param[in] limit Maximum number of seconds (returns anyway when reached).
 *
 * @return Seconds of consecutive positive reads.
 */
static float button_user_count_seconds(float limit)
{
	stopwatch_start();
	while (stopwatch_stop() < limit) {
		if (!button_read_user())
			break;
	}
	return stopwatch_stop();
}

/**
 * @brief Check for any user button response.
 *
 * A user button response can either be an action or no-response.
 *
 * @return The user button response.
 */
enum button_response button_user_response(void)
{
	float readings = button_user_count_seconds(.5);

	if (readings < 0.05)
		return BUTTON_NONE;
	if (readings < 0.3) {
		speaker_play_beeps(1);
		return BUTTON_SHORT;
	}
	led_left_on();
	led_right_on();
	while (button_read_user())
		;
	led_left_off();
	led_right_off();
	speaker_play_beeps(2);
	return BUTTON_LONG;
}

/**
 * @brief Wait for an user button action.
 *
 * This function blocks until an action is detected.
 *
 * @return The action that was detected.
 */
enum button_action button_user_wait_action(void)
{
	enum button_response action;

	while (1) {
		action = button_user_response();
		if (action == BUTTON_NONE)
			continue;
		return (enum button_action)action;
	}
}

/**
 * @brief Warn low battery using speaker sounds.
 */
void speaker_warn_low_battery(void)
{
	music_play('C', 4, 0, 0.05);
	sleep_ticks(50);
	music_play('C', 3, 0, 0.05);
	sleep_ticks(50);
}

/**
 * @brief Notify about an error with a low pitch sustained sound.
 */
void speaker_play_error(void)
{
	music_play('C', 3, 0, 2.);
}

/**
 * @brief Play consecutive beeps.
 *
 * A beep is a short, high tone followed by a short silence.
 *
 * @param[in] beeps Number of beeps to play.
 */
void speaker_play_beeps(uint8_t beeps)
{
	for (int i = 0; i < beeps; i++) {
		music_play('C', 8, 0, 0.05);
		sleep_ticks(50);
	}
}

/**
 * @brief Play three fast, high tones to note a successful operation.
 */
void speaker_play_success(void)
{
	speaker_play_beeps(3);
}

/**
 * @brief Play an epic composition before competition.
 */
void speaker_play_competition(void)
{
	music_play('C', 7, 0, 0.15);
	sleep_seconds(0.15);
	music_play('C', 7, 0, 0.15);
	music_play('G', 7, 0, 0.60);
	sleep_seconds(0.15);
	music_play('G', 7, 0, 0.15);
	music_play('A', 7, 0, 0.15);
	music_play('G', 7, 0, 0.15);
	music_play('F', 7, 0, 0.15);
	music_play('G', 7, 0, 0.45);
}

/**
 * @brief Wait for a close front sensor signal.
 *
 * @param[in] close_distance Distance to be considered as close, in meters.
 */
void wait_front_sensor_close_signal(float close_distance)
{
	while (1) {
		if (get_front_right_distance() < close_distance ||
		    get_front_left_distance() < close_distance)
			break;
	}
}

/**
 * @brief Configure initial search direction for the solver.
 */
void configure_solver_direction(void)
{
	switch (button_user_wait_action()) {
	case BUTTON_SHORT:
		set_search_initial_direction(NORTH);
		led_left_on();
		break;
	case BUTTON_LONG:
		set_search_initial_direction(EAST);
		led_right_on();
		break;
	}
}

/**
 * @brief Select a force level for exploration or run phases.
 *
 * It starts at a minimum defined force and increases that force by steps.
 *
 * @param[in] minimum_force Minimum force.
 * @param[in] force_step Force increase on each step.
 *
 * @return The selected force.
 */
float hmi_configure_force(float minimum_force, float force_step)
{
	uint8_t force = 0;

	while (1) {
		switch (button_user_wait_action()) {
		case BUTTON_SHORT:
			if (force == 10)
				force = 0;
			else
				force += 1;
			repeat_blink(force, 200);
			break;
		case BUTTON_LONG:
			led_left_on();
			led_right_on();
			sleep_ticks(1000);
			return force * force_step + minimum_force;
		}
	}
}
