/*
@file
*/

#include "pwm_driver.h"
#include "nrfx_pwm.h"
#include "boards.h"
#include "nrf_log.h"


// <=============================== PWM0 ================================>

/*
   f(pwm) = f(pwm_clock) / (COUNTERTOP + 1)

*/
#define PWM_TOP_VALUE 19999    // 255 // COUNTERTOP
#define SERVO_PIN 25

static const nrfx_pwm_t m_pwm0 = NRFX_PWM_INSTANCE(0);
static nrf_pwm_values_individual_t pwm_values;

static nrf_pwm_sequence_t const led_seq = {
    .values.p_individual = &pwm_values, .length = NRF_PWM_VALUES_LENGTH(pwm_values), .repeats = 0, .end_delay = 0};

static const nrfx_pwm_config_t pwm_config = {

    .output_pins  = { BSP_LED_1 | NRFX_PWM_PIN_INVERTED,
                      SERVO_PIN | NRFX_PWM_PIN_INVERTED,
                     NRFX_PWM_PIN_NOT_USED,
                     NRFX_PWM_PIN_NOT_USED},
    .base_clock   = NRF_PWM_CLK_1MHz,
    .irq_priority = APP_IRQ_PRIORITY_LOWEST,
    .count_mode   = NRF_PWM_MODE_UP,
    .top_value    = PWM_TOP_VALUE,
    .load_mode    = NRF_PWM_LOAD_INDIVIDUAL,
    .step_mode    = NRF_PWM_STEP_AUTO

};

void pwm_init(void) {
  APP_ERROR_CHECK(nrfx_pwm_init(&m_pwm0, &pwm_config, NULL));
  pwm_values.channel_0 = 0;
  APP_ERROR_CHECK(nrfx_pwm_simple_playback(&m_pwm0, &led_seq, 1, NRFX_PWM_FLAG_LOOP));
}

void pwm_uinit(void) {
  nrfx_pwm_uninit(&m_pwm0);
}

/*
TODO
*/

void on_pwm_evt(uint8_t *duty_cycle) {
  // 0 - 255 mapped to 0 - 19999 (PWM_TOP_VALUE);
  uint16_t duty        = (*duty_cycle * PWM_TOP_VALUE) / 255;
  pwm_values.channel_0 = duty;

  /*
  If PWM_TOP_VALUE = 19999 (for 20 ms period),
then servo’s pulse width range (1–2 ms) corresponds to 1000–2000 ticks.
0ms to 2ms width (0 - 180);
*/
  uint16_t servo_pulse = 1000 + ((*duty_cycle) * (1000)) / 255;
  pwm_values.channel_1 = servo_pulse;

  NRF_LOG_DEBUG("PWM values, led: %d, servo: %d", duty, servo_pulse);
}



