/*
 * init.h
 *
 * Created: 2020-12-28 13:12:11
 *  Author: Branden
 */ 


#ifndef INIT_H_
#define INIT_H_

#define  F_CPU 32000000

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "touch_api.h"

#define TIMER_PER_MS 125
#define INPUT_TIMEOUT_PER_MS 1000

#ifdef QTOUCH_STUDIO_MASKS
extern TOUCH_DATA_T SNS_array[2][2];
extern TOUCH_DATA_T SNSK_array[2][2];
#endif

void init_start_mode_pins();

void init_main_config();

void init_mem_access_config();

#endif /* INIT_H_ */