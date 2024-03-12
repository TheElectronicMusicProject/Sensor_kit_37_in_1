/**
 * @file    main.ino
 * @author  Filippo Graziani
 * @date    12/03/2024
 * @brief   A sample code for the IR obstacle avoidance sensor (KY032)
 * Connections:
 * Sensor       -----       Arduino Uno
 * GND                      GND
 * +                        5V
 * out                      D3
 * EN                       N.C. if the jumper is plugged in
 *
 * @note    There is a infrared LED and a infrared receiver @38kHz.
 * @attention   Avoid changing the frequency setting.
 */

#include "Arduino.h"
#include <stdint.h>

#define     PIN_OUT                 (3)
#define     PIN_LED                 (LED_BUILTIN)
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
    pinMode(PIN_OUT, INPUT);
}   /* setup() */

/**
 * @brief   Loop function.
 * @par     Description
 * Checking the presence of an obstacle.
 * @return  Nothing.
 */
void
loop ()
{
    static bool b_prev_found(digitalRead(PIN_OUT));
    int32_t found(digitalRead(PIN_OUT));

    if (LOW == found)
    {
        if (b_prev_found != found)
        {
            Serial.println("Obstacle!");
            digitalWrite(PIN_LED, 1);

            b_prev_found = found;
        }
    }
    else
    {
        if (b_prev_found != found)
        {
            Serial.println("Free from obstacles");
            digitalWrite(PIN_LED, 0);

            b_prev_found = found;
        }
    }
}   /* loop() */


/*** End of file ***/
