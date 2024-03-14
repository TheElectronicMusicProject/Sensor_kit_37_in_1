/**
 * @file    main.ino
 * @author  Filippo Graziani
 * @date    09/03/2024
 * @brief   A sample code for the button (KY004)
 * Connections:
 * Sensor       -----       Arduino Uno
 * -                        GND
 * [no label]               5V
 * S                        D3
 */

#include "Arduino.h"
#include <stdint.h>

void irq_btn_press();

#define     PIN_BTN                 (3)
#define     PIN_LED_PWM             (10)

static volatile bool b_pressed(false);

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

    pinMode(PIN_LED_PWM, OUTPUT);
    pinMode(PIN_BTN, INPUT);
    attachInterrupt(digitalPinToInterrupt(PIN_BTN), irq_btn_press, FALLING);

}   /* setup() */

/**
 * @brief   Loop function.
 * @par     Description
 * Everytime the button is pressed, the LED changes its PWM value increasing the
 * light until its maximum value, then it comes back to low value.
 * @return  Nothing.
 */
void
loop ()
{
    static uint32_t debounce_time_ms(0);
    static int32_t led_intesity(0);

    if ((true == b_pressed) && (0 == debounce_time_ms))
    {
        detachInterrupt(digitalPinToInterrupt(PIN_BTN));
        
        Serial.println("Button pressed");

        led_intesity += 50;
        led_intesity %= 256;
        analogWrite(PIN_LED_PWM, led_intesity);
        debounce_time_ms = millis();
    }

    if ((0 != debounce_time_ms) && ((millis() - debounce_time_ms) > 300))
    {
        debounce_time_ms = 0;
        b_pressed = false;
        attachInterrupt(digitalPinToInterrupt(PIN_BTN),
                                              irq_btn_press,
                                              FALLING);
    }
}   /* loop() */

/**
 * @brief   Interrupt function.
 * @par     Description
 * Detection of a button pression.
 * @return  Nothing.
 */
void
irq_btn_press ()
{
    b_pressed = true;
}   /* irq_btn_press() */


/*** End of file ***/
