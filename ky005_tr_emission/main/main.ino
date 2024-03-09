/**
 * @file    main.ino
 * @author  Filippo Graziani
 * @date    09/03/2024
 * @brief   A sample code for the infrared transmitter (KY005)
 * Connections:
 * Sensor       -----       Arduino Uno
 * -                        +
 * [no label]               [read note]
 * S                        D5
 *
 * @note    This pin can be used for the connection of GND + resistor (so no
 *          need for an external resistor), but you must insert the missing
 *          resistor (about 220ohm).
 */

#include "Arduino.h"
#include <stdint.h>

#define     PIN_LED                 (5)

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
    randomSeed(analogRead(A3));
}   /* setup() */

/**
 * @brief   Loop function.
 * @par     Description
 * Random switching on/off of the infrared LED.
 * @return  Nothing.
 */
void
loop ()
{
    static uint8_t led_status(0);
    static uint32_t prev_time(millis());
    uint32_t wait_time(random(1, 1000));

    if ((millis() - prev_time) > wait_time)
    {
        Serial.println("LED status is " + String(led_status));
        digitalWrite(PIN_LED, led_status);
        led_status = ~led_status;
        prev_time = millis();
    }
}   /* loop() */


/*** End of file ***/
