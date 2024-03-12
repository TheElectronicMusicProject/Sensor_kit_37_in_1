/**
 * @file    main.ino
 * @author  Filippo Graziani
 * @date    06/03/2024
 * @brief   A sample code for the two-color LED 3mm (KY029)
 * Connections:
 * Sensor       -----       Arduino Uno
 * -                        GND
 * G                        D8
 * R                        D9
 *
 * @attention   Resistors are needed.
 * @note    It is also called bi-led.
 */

#include "Arduino.h"
#include <stdint.h>

#define     PIN_LED_GREEN   (8)
#define     PIN_LED_RED     (9)

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

    pinMode(PIN_LED_GREEN, OUTPUT);
    pinMode(PIN_LED_RED, OUTPUT);
}   /* setup() */

/**
 * @brief   Loop function.
 * @par     Description
 * Tests the fading of the colors of the two-color LED.
 * @return  Nothing.
 */
void
loop ()
{
    Serial.println("Transition Red to green");

    // Red to green.
    //
    for (uint8_t level(0); level < 255; ++level)
    {
        analogWrite(PIN_LED_RED, 255 - level);
        analogWrite(PIN_LED_GREEN, level);
        delay(10);
    }

    Serial.println("Transition green to red");

    // Green to red.
    //
    for (uint8_t level(0); level < 255; ++level)
    {
        analogWrite(PIN_LED_RED, level);
        analogWrite(PIN_LED_GREEN, 255 - level);
        delay(10);
    }
}   /* loop() */


/*** End of file ***/
