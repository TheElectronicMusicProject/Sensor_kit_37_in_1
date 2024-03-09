/**
 * @file    main.ino
 * @author  Filippo Graziani
 * @date    08/03/2024
 * @brief   A sample code for the digital temperature sensor (KY028)
 * Connections:
 * Sensor       -----       Arduino Uno
 * A0                       A0
 * G                        GND
 * +                        5V
 * D0                       D3
 *
 * @note    It is the similar to KY023 but this sensor doesn't show absolute
 *          values, it is a relative measurement to the user-defined
 *          environmental conditions.
 * @note    Check the given analog value at room temperature, then check the
 *          analog value when the component stays at a cricital temperature.
 *          This is the way to detect the temperature variation and alarms.
 */

#include "Arduino.h"
#include <stdint.h>
#include <math.h>

#define     PIN_TEMP    (A0)
#define     PIN_DIGITAL (3)
#define     PIN_LED     (LED_BUILTIN)

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
    pinMode(PIN_DIGITAL, INPUT);
}   /* setup() */

/**
 * @brief   Loop function.
 * @par     Description
 * The analog temperature is read from the analog pin and but the digital pin
 * is used to detect a threshold.
 * @return  Nothing.
 */
void
loop ()
{
    Serial.println(analogRead(PIN_TEMP));

    // Test of a threshold.
    //
    if (1 == digitalRead(PIN_DIGITAL))
    {
        digitalWrite(PIN_LED, 1);
    }
    else
    {
        digitalWrite(PIN_LED, 0);
    }
}   /* loop() */


/*** End of file ***/
