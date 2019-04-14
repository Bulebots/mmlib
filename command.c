#include "command.h"

/**
 * @brief Process a command received.
 *
 * @param[in] string String buffer potentially containing the command.
 */
static void process_command(char *string)
{
	LOG_DEBUG("Processing \"%s\"", string);
	struct control_constants control = get_control_constants();

	if (!strcmp(string, "battery"))
		log_battery_voltage();
	else if (!strcmp(string, "configuration_variables"))
		log_configuration_variables();
	else if (!strcmp(string, "run linear_speed_profile"))
		run_linear_speed_profile();
	else if (!strcmp(string, "run angular_speed_profile"))
		run_angular_speed_profile();
	else if (!strcmp(string, "run static_turn_right_profile"))
		run_static_turn_right_profile();
	else if (!strcmp(string, "run front_sensors_calibration"))
		run_front_sensors_calibration();
	else if (starts_with(string, "move "))
		run_movement_sequence(string);
	else if (starts_with(string, "set micrometers_per_count "))
		set_micrometers_per_count(
		    parse_float(string, RECEIVE_BUFFER_SIZE, 2));
	else if (starts_with(string, "set wheels_separation "))
		set_wheels_separation(
		    parse_float(string, RECEIVE_BUFFER_SIZE, 2));
	else if (starts_with(string, "set max_linear_speed "))
		set_max_linear_speed(
		    parse_float(string, RECEIVE_BUFFER_SIZE, 2));
	else if (starts_with(string, "set kp_linear ")) {
		control.kp_linear = parse_float(string, RECEIVE_BUFFER_SIZE, 2);
		set_control_constants(control);
	} else if (starts_with(string, "set kd_linear ")) {
		control.kd_linear = parse_float(string, RECEIVE_BUFFER_SIZE, 2);
		set_control_constants(control);
	} else if (starts_with(string, "set kp_angular ")) {
		control.kp_angular =
		    parse_float(string, RECEIVE_BUFFER_SIZE, 2);
		set_control_constants(control);
	} else if (starts_with(string, "set kd_angular ")) {
		control.kd_angular =
		    parse_float(string, RECEIVE_BUFFER_SIZE, 2);
		set_control_constants(control);
	} else if (starts_with(string, "set ki_angular_side ")) {
		control.ki_angular_side =
		    parse_float(string, RECEIVE_BUFFER_SIZE, 2);
		set_control_constants(control);
	} else if (starts_with(string, "set ki_angular_front ")) {
		control.ki_angular_front =
		    parse_float(string, RECEIVE_BUFFER_SIZE, 2);
		set_control_constants(control);
	} else if (starts_with(string, "set kp_angular_side ")) {
		control.kp_angular_side =
		    parse_float(string, RECEIVE_BUFFER_SIZE, 2);
		set_control_constants(control);
	} else if (starts_with(string, "set kp_angular_front ")) {
		control.kp_angular_front =
		    parse_float(string, RECEIVE_BUFFER_SIZE, 2);
		set_control_constants(control);
	} else
		LOG_ERROR("Unknown command: `%s`!", string);
}

/**
 * @brief Execute a command received.
 */
void execute_command(void)
{
	char *receive_buffer = get_received_serial_buffer();

	if (!get_received_command_flag() || receive_buffer[0] == '\0')
		return;
	process_command(receive_buffer);
	set_received_command_flag(false);
}
