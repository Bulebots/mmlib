#include "solve.h"

#define RUN_SEQUENCE_LEN (MAZE_AREA + 3)
#define EEPROM_NUM_BYTES_ERASED_CHECKED ((uint8_t)4)
#define EEPROM_BYTE_ERASED_VALUE 255
static char run_sequence[RUN_SEQUENCE_LEN];

/**
 * @brief Move from the current position to the defined target.
 *
 * @param[in] force Maximum force to apply on the tires.
 */
static void go_to_target(float force)
{
	enum step_direction step;
	struct walls_around walls;

	set_distances();
	do {
		if (!current_cell_is_visited()) {
			walls = read_walls();
			update_walls(walls);
			set_distances();
		} else {
			walls = current_walls_around();
		}
#ifdef MMSIM_SIMULATION
		send_state();
#endif
		step = best_neighbor_step(walls);
		move_search_position(step);
		move(step, force);
		if (collision_detected())
			return;
	} while (search_distance() > 0);

	walls = read_walls();
	update_walls(walls);
}

/**
 * @brief Execute the maze exploration.
 *
 * @param[in] force Maximum force to apply on the tires.
 *
 * After reaching the goal, it will try to explore remaining parts until
 * finding an optimal path.
 */
void explore(float force)
{
	uint8_t cell;

	initialize_maze_walls();
	set_search_initial_state();

	while (true) {
		go_to_target(force);
		if (collision_detected())
			return;
		if (search_position() == 0)
			break;
		cell = find_unexplored_interesting_cell();
		set_target_cell(cell);
	}
	stop_middle();
	turn_to_start_position(force);
}

/**
 * @brief Define the movement sequence to be executed on speed runs.
 */
void set_run_sequence(void)
{
	int i = 0;
	enum step_direction step;

	set_search_initial_state();
	set_target_goal();
	set_distances();

	run_sequence[i++] = 'B';
	while (search_distance() > 0) {
		step = best_neighbor_step(current_walls_around());
		switch (step) {
		case FRONT:
			run_sequence[i++] = 'F';
			break;
		case LEFT:
			run_sequence[i++] = 'L';
			break;
		case RIGHT:
			run_sequence[i++] = 'R';
			break;
		default:
			break;
		}
		move_search_position(step);
	}
	while (true) {
		move_search_position(FRONT);
		if (search_distance() != 0)
			break;
		run_sequence[i++] = 'F';
	}
	run_sequence[i++] = 'F';
	run_sequence[i++] = 'S';
	run_sequence[i] = '\0';
}

/**
 * @brief Run from the start to the goal.
 *
 * @param[in] force Maximum force to apply on the tires.
 */
void run(float force)
{
	execute_movement_sequence(run_sequence, force, PATH_DIAGONALS);
}

/**
 * @brief Run back from the goal to the start.
 *
 * @param[in] force Maximum force to apply on the tires.
 */
void run_back(float force)
{
	int length;
	char run_back[MAZE_AREA];
	char translation = '\0';

	length = strlen(run_sequence);
	for (int i = 0; i < length; i++) {
		switch (run_sequence[i]) {
		case 'F':
			translation = 'F';
			break;
		case 'L':
			translation = 'R';
			break;
		case 'R':
			translation = 'L';
			break;
		case 'B':
			translation = 'S';
			break;
		case 'S':
			translation = 'B';
			break;
		default:
			continue;
		}
		run_back[length - i - 1] = translation;
	}
	run_back[length] = '\0';
	execute_movement_sequence(run_back, force, PATH_SAFE);
}

/**
 * @brief Function to save the maze sequence on EEPROM.
 */
void save_maze(void)
{
	uint32_t save_status = 0;

	save_status = eeprom_flash_page(FLASH_EEPROM_ADDRESS_MAZE,
					(uint8_t *)run_sequence, MAZE_AREA);

	if (save_status != RESULT_OK)
		LOG_ERROR("EEPROM save error %" PRIu32, save_status);
}

/**
 * @brief Function to load the maze sequence from EEPROM to static on RAM.
 */
void load_maze(void)
{
	eeprom_read_data(FLASH_EEPROM_ADDRESS_MAZE, MAZE_AREA,
			 (uint8_t *)run_sequence);
}

/**
 * @brief Function to reset the maze sequence on EEPROM.
 */
void reset_maze(void)
{
	uint32_t erase_status = 0;

	erase_status = eeprom_erase_page(FLASH_EEPROM_ADDRESS_MAZE);
	if (erase_status != RESULT_OK)
		LOG_ERROR("EEPROM reset error %" PRIu32, erase_status);
}

/**
 * @brief Function to check if the maze sequence is saved on EEPROM.
 *
 *@return bool
 */
bool maze_is_saved(void)
{
	uint8_t maze_sample[EEPROM_NUM_BYTES_ERASED_CHECKED];

	eeprom_read_data(FLASH_EEPROM_ADDRESS_MAZE,
			 EEPROM_NUM_BYTES_ERASED_CHECKED, maze_sample);

	for (uint8_t iter = 0; iter < EEPROM_NUM_BYTES_ERASED_CHECKED; iter++) {
		if (maze_sample[iter] != EEPROM_BYTE_ERASED_VALUE)
			return true;
	}

	return false;
}
