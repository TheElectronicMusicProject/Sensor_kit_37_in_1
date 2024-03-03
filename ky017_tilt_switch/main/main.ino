/**
 * @file    main.ino
 * @author  Filippo Graziani
 * @date    03/03/2024
 * @brief   A sample code for the tilt switch (KY017)
 * Connections:
 * Sensor       -----       Arduino Uno
 * -                        GND
 * +                        5V
 * S                        D3
 *
 * @warning There is mercury inside the sensor!
 */

#include "Arduino.h"
#include <stdint.h>

void irq_tilt_on();
void irq_tilt_off();

#define     PIN_TILT        (3)

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

    pinMode(PIN_TILT, INPUT);
    attachInterrupt(digitalPinToInterrupt(PIN_TILT), irq_tilt_on, FALLING);

}   /* setup() */

/**
 * @brief   Loop function.
 * @par     Description
 * The LED is turned on if the tilt sensor is enabled, is turned off otherwise.
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
            attachInterrupt(digitalPinToInterrupt(PIN_TILT), irq_tilt_off,
                            RISING);
        }
        else
        {
            Serial.println("Disabled status");
            attachInterrupt(digitalPinToInterrupt(PIN_TILT), irq_tilt_on,
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
irq_tilt_on ()
{
    b_is_active = true;
}   /* irq_tilt_on() */

/**
 * @brief   Interrupt function.
 * @par     Description
 * Detection of the disabled state of the tilt sensor.
 * @return  Nothing.
 */
void
irq_tilt_off ()
{
    b_is_active = false;
}   /* irq_tilt_off() */


/*** End of file ***/
