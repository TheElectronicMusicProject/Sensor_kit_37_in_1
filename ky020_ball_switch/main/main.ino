/**
 * @file    main.ino
 * @author  Filippo Graziani
 * @date    07/03/2024
 * @brief   A sample code for the ball switch (KY020)
 * Connections:
 * Sensor       -----       Arduino Uno
 * -                        GND
 * [no label]               5V
 * S                        D3
 *
 * @note    This is another kind of tilt switch.
 */

#include "Arduino.h"
#include <stdint.h>

void irq_ball_on();
void irq_ball_off();

#define     PIN_BALL_SWITCH (3)
#define     PIN_LED         (LED_BUILTIN)

static volatile bool b_is_active(false);

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

    pinMode(PIN_LED, OUTPUT);
    pinMode(PIN_BALL_SWITCH, INPUT);
    attachInterrupt(digitalPinToInterrupt(PIN_BALL_SWITCH), irq_ball_on,
                    FALLING);
}   /* setup() */

/**
 * @brief   Loop function.
 * @par     Description
 * The LED is turned on if the ball sensor is enabled, is turned off otherwise.
 * @return  Nothing.
 */
void
loop ()
{
    static bool b_prev_active(b_is_active);

    if (b_prev_active != b_is_active)
    {
        if (true == b_is_active)
        {
            Serial.println("Enabled status");
            digitalWrite(PIN_LED, 1);
            attachInterrupt(digitalPinToInterrupt(PIN_BALL_SWITCH),
                            irq_ball_off, RISING);
        }
        else
        {
            Serial.println("Disabled status");
            digitalWrite(PIN_LED, 0);
            attachInterrupt(digitalPinToInterrupt(PIN_BALL_SWITCH), irq_ball_on,
                            FALLING);
        }

        b_prev_active = b_is_active;
    }
}   /* loop() */

/**
 * @brief   Interrupt function.
 * @par     Description
 * Detection of the enabled state of the tilt sensor.
 * @return  Nothing.
 */
void
irq_ball_on ()
{
    b_is_active = true;
}   /* irq_ball_on() */

/**
 * @brief   Interrupt function.
 * @par     Description
 * Detection of the disabled state of the tilt sensor.
 * @return  Nothing.
 */
void
irq_ball_off ()
{
    b_is_active = false;
}   /* irq_ball_off() */


/*** End of file ***/
