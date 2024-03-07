/**
 * @file    main.ino
 * @author  Filippo Graziani
 * @date    07/03/2024
 * @brief   A sample code for the laser emit LED (KY008)
 * Connections:
 * Sensor       -----       Arduino Uno
 * S                        D3
 * [no label]               5V
 * -                        GND
 *
 * @attention   This light is very dangerous!
 */

#include "Arduino.h"
#include <stdint.h>

#define     PIN_LED_LASER   (3)

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

    pinMode(PIN_LED_LASER, OUTPUT);
}   /* setup() */

/**
 * @brief   Loop function.
 * @par     Description
 * Tests the switch on/off of the laser light emitter.
 * @return  Nothing.
 */
void
loop ()
{
    static uint32_t prev_time(millis());
    static uint8_t led_status(0);

    if ((millis() - prev_time) > 500)
    {
        Serial.println("Led is -> " + String(led_status));
        digitalWrite(PIN_LED_LASER, led_status);
        prev_time = millis();
        led_status = ~led_status;
    }
}   /* loop() */


/*** End of file ***/
