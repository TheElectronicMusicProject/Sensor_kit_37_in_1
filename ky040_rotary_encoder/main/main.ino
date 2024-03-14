/**
 * @file    main.ino
 * @author  Filippo Graziani
 * @date    14/03/2024
 * @brief   A sample code for the rotary encoder (KY040)
 * Connections:
 * Sensor       -----       Arduino Uno
 * GND                      GND
 * +                        5V
 * SW                       D4
 * DT                       D2
 * CLK                      D3
 *
 * @note    DT and CLK are already connected with pull-up resistors (10kohm).
 */

#include "Arduino.h"
#include <stdint.h>

#define     PIN_SW                  (2)
#define     PIN_DT                  (4)
#define     PIN_CLK                 (3)
#define     PIN_PWM_LED             (10)

#define     USE_INTERRUPT           (0)

#if 1 == USE_INTERRUPT
static void irq_btn_press();
static void irq_enc();

static volatile bool gb_bnt_press(false);
static volatile bool gb_enc_rotate(false);
#endif /* USE_INTERRUPT */

/**
 * @brief   Setup function.
 * @par     Description
 * Setting of the Arduino's pins and the serial port.
 * @return  Nothing.
 */
void
setup ()
{
    Serial.begin(9600);

    while (!Serial)
    {
        /* Do nothing */
    }

    pinMode(PIN_PWM_LED, OUTPUT);
    pinMode(PIN_SW, INPUT_PULLUP);
    pinMode(PIN_DT, INPUT);
    pinMode(PIN_CLK, INPUT);
#if 1 == USE_INTERRUPT
    attachInterrupt(digitalPinToInterrupt(PIN_SW), irq_btn_press, FALLING);
    attachInterrupt(digitalPinToInterrupt(PIN_CLK), irq_enc, FALLING);
#endif /* USE_INTERRUPT */
}   /* setup() */

/**
 * @brief   Loop function.
 * @par     Description
 * Managing the encoder to regulate the PWM of a LED.
 * The button of the encoder turns off and on the LED.
 * Rotating the encoder clockwise the light increases.
 * Rotating the encoder counterclockwise the light decreases.
 * @return  Nothing.
 */
void
loop ()
{
    int32_t curr_dt(digitalRead(PIN_DT));
#if 0 == USE_INTERRUPT
    int32_t curr_clk(digitalRead(PIN_CLK));
    int32_t curr_sw(digitalRead(PIN_SW));
    static int32_t prev_clk(curr_clk);
    static int32_t prev_sw(curr_sw);
#endif /* USE_INTERRUPT */
    static int32_t val(0);
    static bool b_led_off(false);
    static int32_t pwm_val(0);
    static uint32_t btn_debounce_time_ms(0);

#if 1 == USE_INTERRUPT
    if (true == gb_enc_rotate)
    {
        gb_enc_rotate = false;
#else
    if ((LOW == curr_clk) && (HIGH == prev_clk))
    {
#endif /* USE_INTERRUPT */
        if (HIGH == curr_dt)
        {
            --val;
        }
        else
        {
            ++val;
        }

        Serial.println("Rotation " + String(val));

        pwm_val = (val * 10) % 256;

        if (pwm_val < 0)
        {
            pwm_val = -pwm_val;
        }
    }

#if 1 == USE_INTERRUPT
    if ((true == gb_bnt_press) && (0 == btn_debounce_time_ms))
    {
        detachInterrupt(digitalPinToInterrupt(PIN_SW));
        Serial.println("Button pressed");
        btn_debounce_time_ms = millis();
        b_led_off = !b_led_off;
    }
#else
    if ((prev_sw != curr_sw) && (LOW == curr_sw) && (0 == btn_debounce_time_ms))
    {
        Serial.println("Button pressed");
        btn_debounce_time_ms = millis();
        b_led_off = !b_led_off;
    }
#endif /* USE_INTERRUPT */

    if (false == b_led_off)
    {
        analogWrite(PIN_PWM_LED, pwm_val);
    }
    else
    {
        analogWrite(PIN_PWM_LED, 255);
    }

#if 0 == USE_INTERRUPT
    prev_clk = curr_clk;
    prev_sw = curr_sw;
#endif /* USE_INTERRUPT */

    if ((0 != btn_debounce_time_ms) &&
        ((millis() - btn_debounce_time_ms) > 1500))
    {
        btn_debounce_time_ms = 0;
#if 1 == USE_INTERRUPT
        gb_bnt_press = false;
        attachInterrupt(digitalPinToInterrupt(PIN_SW), irq_btn_press,
                        FALLING);
#endif /* USE_INTERRUPT */
    }
}   /* loop() */

#if 1 == USE_INTERRUPT
/**
 * @brief   Interrupt function.
 * @par     Description
 * Detection of the button pressure.
 * @return  Nothing.
 */
static void
irq_btn_press ()
{
    gb_bnt_press = true;
}   /* irq_btn_press() */

/**
 * @brief   Interrupt function.
 * @par     Description
 * Detection of a falling edge on the CLK pin of the encoder.
 * @return  Nothing.
 */
static void
irq_enc ()
{
    gb_enc_rotate = true;
}   /* irq_enc() */
#endif /* USE_INTERRUPT */


/*** End of file ***/
