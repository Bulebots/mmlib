#include "move.h"

static int32_t current_cell_start_micrometers;
/* Angular acceleration is defined in radians per second squared. */
static float angular_acceleration;

/**
 * @brief Return the current robot shift inside the cell, in meters.
 *
 * The shift is the traveled distance since the start of the cell.
 */
static float _current_cell_shift(void)
{
	return (float)(get_encoder_average_micrometers() -
		       current_cell_start_micrometers) /
	       MICROMETERS_PER_METER;
}

/**
 * @brief Mark the beginning of a new cell.
 *
 * It should be executed right after entering a new cell.
 *
 * Takes into account a possible front-wall longitudinal correction.
 */
static void _entered_next_cell(void)
{
	int32_t front_wall_correction;

	current_cell_start_micrometers = get_encoder_average_micrometers();
	if (front_wall_detection()) {
		front_wall_correction =
		    (int32_t)((get_front_wall_distance() - CELL_DIMENSION) *
			      MICROMETERS_PER_METER);
		current_cell_start_micrometers += front_wall_correction;
	}
	led_left_toggle();
}

/**
 * @brief Initialize mouse position.
 *
 * Assumes the mouse tail is initially touching a wall.
 */
void set_starting_position(void)
{
	current_cell_start_micrometers =
	    get_encoder_average_micrometers() -
	    MOUSE_START_SHIFT * MICROMETERS_PER_METER;
}

/**
 * @brief Calculate the required micrometers to reach a given speed.
 *
 * This functions assumes the current speed is the target speed and takes into
 * account the configured linear deceleration.
 *
 * @param[in] speed Target speed.

 * @return The required number of micrometers to reach the target speed.
 */
int32_t required_micrometers_to_speed(float speed)
{
	float acceleration;
	float current_speed = get_ideal_linear_speed();

	acceleration = (current_speed > speed) ? -get_linear_deceleration()
					       : get_linear_acceleration();

	return (int32_t)((speed * speed - current_speed * current_speed) /
			 (2 * acceleration) * MICROMETERS_PER_METER);
}

/**
 * @brief Calculate the required time to reach a given speed, in seconds.
 *
 * This functions assumes the current speed is the target speed and takes into
 * account the configured linear deceleration.
 */
float required_time_to_speed(float speed)
{
	float acceleration;
	float target_speed = get_target_linear_speed();

	acceleration = (target_speed > speed) ? -get_linear_deceleration()
					      : get_linear_acceleration();

	return (speed - target_speed) / acceleration;
}

/**
 * @brief Calculate the required ticks to reach a given speed, in ticks.
 *
 * This functions assumes the current speed is the target speed and takes into
 * account the configured linear deceleration.
 */
uint32_t required_ticks_to_speed(float speed)
{
	float required_seconds = required_time_to_speed(speed);

	return (uint32_t)(required_seconds * SYSTICK_FREQUENCY_HZ);
}

/**
 * @brief Reach a target position at a target speed.
 *
 * @param[in] start Starting point, in micrometers.
 * @param[in] distance Distance to travel, in meters, from the starting point.
 * @param[in] speed Target speed, in meters per second.
 */
void target_straight(int32_t start, float distance, float speed)
{
	int32_t target_distance;

	set_ideal_angular_speed(0.);

	target_distance = start + (int32_t)(distance * MICROMETERS_PER_METER);
	if (distance > 0) {
		set_target_linear_speed(get_max_linear_speed());
		while (get_encoder_average_micrometers() <
		       target_distance - required_micrometers_to_speed(speed))
			;
	} else {
		set_target_linear_speed(-get_max_linear_speed());
		while (get_encoder_average_micrometers() >
		       target_distance - required_micrometers_to_speed(speed))
			;
	}
	set_target_linear_speed(speed);
	if (speed == 0.) {
		while (get_ideal_linear_speed() != speed)
			;
	} else {
		while (get_encoder_average_micrometers() < target_distance)
			;
	}
}

/**
 * @brief Reach a target position at a target speed on a diagonal.
 *
 * @param[in] start Starting point, in micrometers.
 * @param[in] distance Distance to travel, in meters, from the starting point.
 * @param[in] control_distance Distance with control, in meters, from the
 * starting point.
 * @param[in] speed Target speed, in meters per second.
 */
static void target_straight_diagonal(int32_t start, float distance,
				     float control_distance, float speed)
{
	int32_t target_distance, target_control_distance;

	set_ideal_angular_speed(0.);
	target_distance = start + (int32_t)(distance * MICROMETERS_PER_METER);
	target_control_distance =
	    start + (int32_t)(control_distance * MICROMETERS_PER_METER);
	diagonal_sensors_control(true);
	set_target_linear_speed(get_max_linear_speed());
	while (get_encoder_average_micrometers() <
	       target_distance - required_micrometers_to_speed(speed)) {
		if (get_encoder_average_micrometers() > target_control_distance)
			diagonal_sensors_control(false);
	};
	set_target_linear_speed(speed);
	while (get_encoder_average_micrometers() < target_distance) {
		if (get_encoder_average_micrometers() > target_control_distance)
			diagonal_sensors_control(false);
	};
}

/**
 * @brief Wait until the robot is perpendicular with respect to the front wall.
 *
 * @param[in] error Allowed error, in meters.
 */
static void wait_front_perpendicular(float error)
{
	int i;
	float average = 0.;

	while (true) {
		for (i = 0; i < 20; i++) {
			average += get_front_sensors_error();
			sleep_ticks(2);
		}
		average /= 20;
		if (average < error)
			break;
	}
}

/**
 * @brief Keep a specified distance from the front wall.
 *
 * @param[in] distance Distance to keep from the front wall, in meters.
 */
void keep_front_wall_distance(float distance)
{
	int i;
	float diff;
	float front_wall_distance;

	if (!front_wall_detection())
		return;

	set_linear_acceleration(get_linear_acceleration() / 2.);
	set_linear_deceleration(get_linear_deceleration() / 2.);

	while (true) {
		front_sensors_control(true);
		side_sensors_close_control(false);
		side_sensors_far_control(false);

		wait_front_perpendicular(KEEP_FRONT_DISTANCE_TOLERANCE);

		front_wall_distance = 0.;
		for (i = 0; i < 20; i++) {
			front_wall_distance += get_front_wall_distance();
			sleep_ticks(2);
		}
		front_wall_distance /= 20;
		diff = front_wall_distance - distance;
		if (fabsf(diff) < KEEP_FRONT_DISTANCE_TOLERANCE)
			break;
		target_straight(get_encoder_average_micrometers(), diff, 0.);
	}

	set_linear_acceleration(get_linear_acceleration() * 2.);
	set_linear_deceleration(get_linear_deceleration() * 2.);

	disable_walls_control();
	reset_control_all();
}

/**
 * @brief Move straight and stop at the end of the current cell.
 */
void stop_end(void)
{
	front_sensors_control(true);
	side_sensors_close_control(true);
	side_sensors_far_control(false);
	target_straight(current_cell_start_micrometers, CELL_DIMENSION, 0.);
	disable_walls_control();
	reset_control_errors();
	_entered_next_cell();
}

/**
 * @brief Move straight and stop when the head would touch the front wall.
 */
void stop_head_front_wall(void)
{
	float distance = CELL_DIMENSION - WALL_WIDTH / 2. - MOUSE_HEAD;

	front_sensors_control(true);
	side_sensors_close_control(true);
	side_sensors_far_control(false);
	target_straight(current_cell_start_micrometers, distance, 0.);
	disable_walls_control();
	reset_control_errors();
}

/**
 * @brief Move straight and stop at the middle of the current cell.
 */
void stop_middle(void)
{
	float distance = CELL_DIMENSION / 2.;

	front_sensors_control(true);
	side_sensors_close_control(true);
	side_sensors_far_control(false);
	target_straight(current_cell_start_micrometers, distance, 0.);
	disable_walls_control();
	reset_control_errors();
}

/**
 * @brief Turn back (180-degree turn) and correct with front walls if possible.
 *
 * @param[in] force Maximum force to apply on the tires.
 */
void turn_back(float force)
{
	int direction_sign;

	if (get_front_wall_distance() < CELL_DIMENSION)
		keep_front_wall_distance(CELL_DIMENSION / 2.);
	disable_walls_control();
	direction_sign = (int)(rand() % 2) * 2 - 1;
	inplace_turn(direction_sign * PI, force);

	current_cell_start_micrometers =
	    get_encoder_average_micrometers() -
	    (CELL_DIMENSION / 2. + SHIFT_AFTER_180_DEG_TURN) *
		MICROMETERS_PER_METER;
}

/**
 * @brief Turn back (180-degree turn) to a starting position.
 *
 * @param[in] force Maximum force to apply on the tires.
 */
void turn_to_start_position(float force)
{
	float distance;

	set_linear_acceleration(get_linear_acceleration() / 4.);
	set_linear_deceleration(get_linear_deceleration() / 4.);

	turn_back(force);
	distance = MOUSE_START_SHIFT - _current_cell_shift();
	target_straight(get_encoder_average_micrometers(), distance, 0.);

	set_linear_acceleration(get_linear_acceleration() * 4.);
	set_linear_deceleration(get_linear_deceleration() * 4.);

	disable_walls_control();
	reset_control_all();
	enable_motor_control();
	drive_break();
}

/**
 * @brief Move front into the next cell.
 */
void move_front(void)
{
	front_sensors_control(true);
	side_sensors_close_control(true);
	side_sensors_far_control(false);
	target_straight(current_cell_start_micrometers, CELL_DIMENSION,
			get_max_linear_speed());
	_entered_next_cell();
}

/**
 * @brief Move front a defined distance ending at a defined speed.
 *
 * @param[in] distance Distance to travel.
 * @param[in] end_linear_speed Speed at which to end the movement.
 */
void parametric_move_front(float distance, float end_linear_speed)
{
	target_straight(get_encoder_average_micrometers(), distance,
			end_linear_speed);
}

/**
 * @brief Move diagonal a defined control and total distance ending at a
 * defined speed.
 *
 * @param[in] distance Distance to travel.
 * @param[in] control_distance Distance with control enabled
 * @param[in] end_linear_speed Speed at which to end the movement.
 */
void parametric_move_diagonal(float distance, float control_distance,
			      float end_linear_speed)
{
	target_straight_diagonal(get_encoder_average_micrometers(), distance,
				 control_distance, end_linear_speed);
}

/**
 * @brief Move left or right into the next cell.
 *
 * @param[in] movement Turn direction (left or right).
 * @param[in] force Maximum force to apply on the tires.
 */
void move_side(enum movement turn, float force)
{
	front_sensors_control(true);
	side_sensors_close_control(true);
	side_sensors_far_control(true);
	target_straight(current_cell_start_micrometers,
			get_move_turn_before(turn),
			get_move_turn_linear_speed(turn, force));
	disable_walls_control();
	speed_turn(turn, force);
	front_sensors_control(true);
	side_sensors_close_control(true);
	side_sensors_far_control(true);
	target_straight(get_encoder_average_micrometers(),
			get_move_turn_after(turn), get_max_linear_speed());
	_entered_next_cell();
}

/**
 * @brief Move back into the previous cell.
 *
 * @param[in] force Maximum force to apply on the tires.
 */
void move_back(float force)
{
	stop_middle();
	turn_back(force);
	move_front();
}

/**
 * @brief Move into the next cell according to a movement direction.
 *
 * @param[in] direction Movement direction.
 * @param[in] force Maximum force to apply on the tires.
 */
void move(enum step_direction direction, float force)
{
	if (direction == LEFT)
		move_side(MOVE_LEFT, force);
	else if (direction == RIGHT)
		move_side(MOVE_RIGHT, force);
	else if (direction == FRONT)
		move_front();
	else if (direction == BACK)
		move_back(force);
	else
		stop_middle();
}

/**
 * @brief Execute an in-place turn.
 *
 * @param[in] radians Radians to turn (positive means left).
 * @param[in] force Maximum force to apply while turning.
 */
void inplace_turn(float radians, float force)
{
	int turn_sign;
	int32_t start;
	int32_t current;
	float time;
	float angular_velocity;
	float max_angular_velocity;
	float factor;
	float arc;
	float transition;
	float duration;
	float transition_angle;

	turn_sign = sign(radians);
	radians = fabsf(radians);
	angular_acceleration =
	    force * MOUSE_WHEELS_SEPARATION / MOUSE_MOMENT_OF_INERTIA;
	max_angular_velocity = sqrt(radians / 2 * angular_acceleration);
	if (max_angular_velocity > MOUSE_MAX_ANGULAR_VELOCITY)
		max_angular_velocity = MOUSE_MAX_ANGULAR_VELOCITY;

	duration = max_angular_velocity / angular_acceleration * PI;
	transition_angle = duration * max_angular_velocity / PI;
	arc = (radians - 2 * transition_angle) / max_angular_velocity;
	transition = duration / 2;
	max_angular_velocity = turn_sign * max_angular_velocity;

	set_target_linear_speed(get_ideal_linear_speed());
	disable_walls_control();
	start = get_clock_ticks();
	while (true) {
		current = get_clock_ticks();
		time = (float)(current - start) / SYSTICK_FREQUENCY_HZ;
		if (time >= 2 * transition + arc)
			break;
		angular_velocity = max_angular_velocity;
		if (time < transition) {
			factor = time / transition;
			angular_velocity *= sin(factor * PI / 2);
		} else if (time >= transition + arc) {
			factor = (time - arc) / transition;
			angular_velocity *= sin(factor * PI / 2);
		}
		set_ideal_angular_speed(angular_velocity);
	}
	set_ideal_angular_speed(0);
}

/**
 * @brief Execute a movement sequence.
 *
 * The sequence is a raw/sharp path, which will be smoothed before execution.
 *
 * @param[in] sequence Sequence of raw movements to execute.
 * @param[in] force Maximum force to apply on the tires.
 * @param[in] language Language to use for the raw-to-smooth path translation.
 */
void execute_movement_sequence(char *sequence, float force,
			       enum path_language language)
{
	int i = 0;
	int many = 0;
	char movement;
	float distance = 0;
	enum movement smooth_path[MAX_SMOOTH_PATH_LEN];

	make_smooth_path(sequence, smooth_path, language);
	while (true) {
		movement = smooth_path[i++];
		switch (movement) {
		case MOVE_START:
			distance = -MOUSE_START_SHIFT;
			break;
		case MOVE_FRONT:
		case MOVE_DIAGONAL:
			many = 0;
			while (true) {
				many += 1;
				if (smooth_path[i] != movement)
					break;
				i++;
			}
			if (movement == MOVE_FRONT)
				distance += many * CELL_DIMENSION;
			else
				distance += many * CELL_DIAGONAL;
			break;
		case MOVE_LEFT:
		case MOVE_RIGHT:
		case MOVE_LEFT_90:
		case MOVE_RIGHT_90:
		case MOVE_LEFT_180:
		case MOVE_RIGHT_180:
		case MOVE_LEFT_TO_45:
		case MOVE_RIGHT_TO_45:
		case MOVE_LEFT_TO_135:
		case MOVE_RIGHT_TO_135:
			distance += get_move_turn_before(movement);
			side_sensors_close_control(true);
			side_sensors_far_control(false);
			parametric_move_front(
			    distance,
			    get_move_turn_linear_speed(movement, force));
			speed_turn(movement, force);
			distance = get_move_turn_after(movement);
			break;
		case MOVE_LEFT_FROM_45:
		case MOVE_RIGHT_FROM_45:
		case MOVE_LEFT_FROM_135:
		case MOVE_RIGHT_FROM_135:
		case MOVE_LEFT_DIAGONAL:
		case MOVE_RIGHT_DIAGONAL:
			distance += get_move_turn_before(movement);
			side_sensors_close_control(false);
			side_sensors_far_control(false);
			parametric_move_diagonal(
			    distance, (distance - CELL_DIAGONAL * 2),
			    get_move_turn_linear_speed(movement, force));
			speed_turn(movement, force);
			distance = get_move_turn_after(movement);
			break;
		case MOVE_STOP:
			distance -= CELL_DIMENSION / 2;
			side_sensors_close_control(true);
			side_sensors_far_control(false);
			parametric_move_front(distance, 0.);
			turn_to_start_position(force);
			speaker_play_success();
			break;
		case MOVE_END:
			return;
		default:
			LOG_ERROR("Unable to process command [%d]!", movement);
			return;
		}
		if (collision_detected()) {
			LOG_ERROR("Collision detected!");
			return;
		}
	}
}
