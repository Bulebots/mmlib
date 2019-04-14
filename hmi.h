#ifndef __HMI_H
#define __HMI_H

#include "mmlib/clock.h"
#include "mmlib/music.h"
#include "mmlib/walls.h"

#include "buttons.h"
#include "leds.h"

enum button_response { BUTTON_NONE = 0, BUTTON_SHORT, BUTTON_LONG };
enum button_action { CLICK_SHORT = BUTTON_SHORT, CLICK_LONG = BUTTON_LONG };

void repeat_blink(uint8_t count, uint16_t time);
void blink_collision(void);
void speaker_warn_low_battery(void);
void speaker_play_error(void);
void speaker_play_beeps(uint8_t beeps);
void speaker_play_success(void);
void speaker_play_competition(void);
enum button_response button_user_response(void);
enum button_action button_user_wait_action(void);
void wait_front_sensor_close_signal(float close_distance);
void configure_solver_direction(void);
float hmi_configure_force(float minimum_force, float force_step);

#endif /* __HMI_H */
