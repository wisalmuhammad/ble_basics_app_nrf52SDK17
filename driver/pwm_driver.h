/*
@file Header file for PWM instance
* Author : Wisal Muhammad
* Date : 19/11/2025
*/

#ifndef  _PWM_H
#define  _PWM_H

#include "app_config.h"

#if APP_ENABLE_PWM

void pwm_init(void);
void pwm_uinit(void);

#endif
#endif