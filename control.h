#ifndef __CONTROL_H
#define __CONTROL_H

#include "mmlib/encoder.h"
#include "mmlib/hmi.h"
#include "mmlib/speed.h"
#include "mmlib/walls.h"

#include "motor.h"
#include "setup.h"

void side_sensors_control(bool value);
void front_sensors_control(bool value);
void diagonal_sensors_control(bool value);
void enable_walls_control(void);
void disable_walls_control(void);
bool collision_detected(void);
void reset_collision_detection(void);
void reset_control_errors(void);
void reset_control_speed(void);
void reset_control_all(void);
void enable_motor_control(void);
void disable_motor_control(void);
void reset_motion(void);
int32_t get_left_pwm(void);
int32_t get_right_pwm(void);
float get_target_linear_speed(void);
float get_ideal_linear_speed(void);
float get_ideal_angular_speed(void);
float get_measured_linear_speed(void);
float get_measured_angular_speed(void);
void motor_control(void);
void set_target_linear_speed(float speed);
void set_ideal_angular_speed(float speed);
void update_ideal_linear_speed(void);

#endif /* __CONTROL_H */
