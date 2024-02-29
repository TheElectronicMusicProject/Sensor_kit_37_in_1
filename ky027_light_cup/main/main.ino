/**
 * @file    main.ino
 * @author  Filippo Graziani
 * @date    29/02/2024
 * @brief   A sample code for the light cup sensor (KY027)
 * Connections:
 * Sensor       -----       Arduino Uno
 * L                        GND
 * -                        5V
 * S                        D3
 * +                        D4
 *
 * It is a tilt sensor with a LED. The labels on the PCB are wrong.
 * The led doesn't depend on the tilt sensor.
 * @warning There is mercury inside the sensor!
 */

#include "Arduino.h"
#include <stdint.h>

void irq_tilt_on();
void irq_tilt_off();

#define     PIN_TILT        (3)
#define     PIN_LED         (4)

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
            digitalWrite(PIN_LED, 1);
            Serial.println("Enabled status");
            attachInterrupt(digitalPinToInterrupt(PIN_TILT), irq_tilt_off,
                            RISING);
        }
        else
        {
            digitalWrite(PIN_LED, 0);
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
