#ifndef __WALLS_H
#define __WALLS_H

#include "mmlib/clock.h"
#include "mmlib/search.h"

#include "detection.h"

void update_distance_readings(uint16_t *on, uint16_t *off);
float get_side_left_distance(void);
float get_side_right_distance(void);
float get_front_left_distance(void);
float get_front_right_distance(void);
float get_side_sensors_error(void);
float get_front_sensors_error(void);
float get_front_wall_distance(void);
bool front_wall_detection(void);
bool right_wall_detection(void);
bool left_wall_detection(void);
struct walls_around read_walls(void);
void side_sensors_calibration(void);

#endif /* __WALLS_H */
