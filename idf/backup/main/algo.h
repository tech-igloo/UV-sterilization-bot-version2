#ifndef ALGO_H
#define ALGO_H

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "esp_err.h"


void init_pid();
void init_pwm();
void move_forward();
void move_left();
void move_right();
void move_back();
void move_stop();
esp_err_t convert_paths(int n);

#endif
