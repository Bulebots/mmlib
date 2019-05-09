#ifndef __SPEED_H
#define __SPEED_H

#include "math.h"

#include "mmlib/common.h"
#include "mmlib/control.h"
#include "mmlib/move.h"
#include "mmlib/path.h"

#include "config.h"
#include "setup.h"

float get_max_force(void);
void set_max_force(float value);
float get_linear_acceleration(void);
float get_linear_deceleration(void);
float get_max_linear_speed(void);
void set_max_linear_speed(float value);
void kinematic_configuration(float force, bool run);
float get_move_turn_before(enum movement move);
float get_move_turn_after(enum movement move);
float get_move_turn_linear_speed(enum movement turn_type, float force);

void speed_turn(enum movement turn_type, float force);

#endif /* __SPEED_H */
