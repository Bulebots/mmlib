#ifndef __MOVE_H
#define __MOVE_H

#include <math.h>

#include "mmlib/clock.h"
#include "mmlib/control.h"
#include "mmlib/hmi.h"
#include "mmlib/logging.h"
#include "mmlib/path.h"
#include "mmlib/search.h"
#include "mmlib/speed.h"
#include "mmlib/walls.h"

#include "motor.h"
#include "setup.h"

void set_starting_position(void);
int32_t required_micrometers_to_speed(float speed);
float required_time_to_speed(float speed);
uint32_t required_ticks_to_speed(float speed);
void target_straight(int32_t start, float distance, float speed);
void keep_front_wall_distance(float distance);
void stop_end(void);
void stop_head_front_wall(void);
void stop_middle(void);
void turn_back(float force);
void turn_to_start_position(float force);
void move_front(void);
void parametric_move_front(float distance, float end_linear_speed);
void parametric_move_diagonal(float distance, float control_distance,
			      float end_linear_speed);
void move_side(enum movement turn, float force);
void move_back(float force);
void move(enum step_direction direction, float force);
void inplace_turn(float radians, float force);
void execute_movement_sequence(char *sequence, float force,
			       enum path_language language);

#endif /* __MOVE_H */
