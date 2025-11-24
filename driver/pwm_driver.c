/*
@file PWM driver initilization
* Further info @link https://docs.nordicsemi.com/bundle/ps_nrf52832/page/pwm.html
* Author : Wisal Muhammad
* Date : 19/11/2025
*/


#include "pwm_driver.h"
#include "boards.h"
#include "nrf_log.h"
#include "nrfx_pwm.h"

#if APP_ENABLE_PWM

// <=============================== PWM0 ================================>

/*
   f(pwm) = f(pwm_clock) / (COUNTERTOP + 1)

*/

// ---------------------------------------------------------------------------
// PWM configuration constants
// PWM_TOP_VALUE determines the PWM period. With 1 MHz clock, 19999 → 20 ms.
// Perfect for servo control (50 Hz).
// ---------------------------------------------------------------------------

#define PWM_TOP_VALUE 19999    // 255 // COUNTERTOP
#define SERVO_PIN 25

// ---------------------------------------------------------------------------
// Declare PWM instance (PWM0) and individual PWM value container.
// `pwm_values` holds 4 channels (0–3), but we use channel_0 and channel_1.
// ---------------------------------------------------------------------------
static const nrfx_pwm_t m_pwm0 = NRFX_PWM_INSTANCE(0);
static nrf_pwm_values_individual_t pwm_values;

// ---------------------------------------------------------------------------
// PWM sequence structure:
// - Points to `pwm_values`.
// - load_mode = individual, so each channel has its own duty.
// - The sequence is played in LOOP mode inside pwm_init().
// ---------------------------------------------------------------------------
static nrf_pwm_sequence_t const led_seq = {
    .values.p_individual = &pwm_values, .length = NRF_PWM_VALUES_LENGTH(pwm_values), .repeats = 0, .end_delay = 0};

// ---------------------------------------------------------------------------
// PWM configuration structure:
// - Channel 0 → LED pin (inverted, because BSP LEDs are active-low)
// - Channel 1 → Servo pin
// - Base clock = 1 MHz, so each tick = 1 µs
// - top_value = 19999 → 20 ms period (50 Hz)
// - load_mode = individual (each channel independent)
// ---------------------------------------------------------------------------

static const nrfx_pwm_config_t pwm_config = {

    .output_pins  = {BSP_LED_1 | NRFX_PWM_PIN_INVERTED,
                     SERVO_PIN,
                     NRFX_PWM_PIN_NOT_USED,
                     NRFX_PWM_PIN_NOT_USED},
    .base_clock   = NRF_PWM_CLK_1MHz,
    .irq_priority = APP_IRQ_PRIORITY_LOWEST,
    .count_mode   = NRF_PWM_MODE_UP,
    .top_value    = PWM_TOP_VALUE,
    .load_mode    = NRF_PWM_LOAD_INDIVIDUAL,
    .step_mode    = NRF_PWM_STEP_AUTO

};

// ---------------------------------------------------------------------------
// Initialize PWM:
// - Starts PWM0 with the config above.
// - Channel 0 (LED) starts at 0%.
// - Channel 1 (servo) starts at 1500 µs (center position).
// - Sequence repeats forever in loop.
// ---------------------------------------------------------------------------
void pwm_init(void) {
  APP_ERROR_CHECK(nrfx_pwm_init(&m_pwm0, &pwm_config, NULL));
  pwm_values.channel_0 = 0;
  pwm_values.channel_1 = 1500;
  APP_ERROR_CHECK(nrfx_pwm_simple_playback(&m_pwm0, &led_seq, 1, NRFX_PWM_FLAG_LOOP));
}

// ---------------------------------------------------------------------------
// Stop and uninitialize PWM instance.
// ---------------------------------------------------------------------------
void pwm_uinit(void) {
  nrfx_pwm_uninit(&m_pwm0);
}


// ---------------------------------------------------------------------------
// PWM update event handler:
// Called when BLE writes a PWM value (0–255).
// - Maps 0–255 → 0–19999 for LED brightness.
// - Maps 0–255 → 1000–2000 µs for servo angle.
// - Writes updated values to channels.
// ---------------------------------------------------------------------------
void on_pwm_evt(uint8_t duty_cycle) {
  NRF_LOG_INFO("PWM duty cycle value: %d", duty_cycle);
  // 0 - 255 mapped to 0 - 19999 (PWM_TOP_VALUE);
  uint16_t duty        = (duty_cycle * PWM_TOP_VALUE) / 255;
  pwm_values.channel_0 = duty;

/*
    Servo pulse width mapping:
    - Servo pulse ranges 1.0 ms → 2.0 ms (1000–2000 ticks)
    - *duty_cycle (0–255) → 1000–2000 µs
  */
  uint16_t servo_pulse = 1000 + ((duty_cycle) * (1000)) / 255;
  // PWM inverts output (active-low), so actual compare value is reversed
  pwm_values.channel_1 = PWM_TOP_VALUE - servo_pulse;

  NRF_LOG_DEBUG("PWM values, led: %d, servo: %d", duty, servo_pulse);
}

#endif 