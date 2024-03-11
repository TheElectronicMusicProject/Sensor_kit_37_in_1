/**
 * @file    main.ino
 * @author  Filippo Graziani
 * @date    11/03/2024
 * @brief   A sample code for the active buzzer (KY012)
 * Connections:
 * Sensor       -----       Arduino Uno
 * -                        GND
 * [no label]               N.C.
 * +                        D3
 */

#include "Arduino.h"
#include <stdint.h>

#define     PIN_BUZZER              (3)
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
    pinMode(PIN_BUZZER, OUTPUT);
}   /* setup() */

/**
 * @brief   Loop function.
 * @par     Description
 * Turning on and off the buzzer.
 * @return  Nothing.
 */
void
loop ()
{
    static uint32_t prev_time(millis());
    static uint32_t status(0);

    if ((millis() - prev_time) > 100)
    {
        digitalWrite(PIN_BUZZER, status);
        digitalWrite(PIN_LED, status);
        status = ~status;

        prev_time = millis();
    }
}   /* loop() */


/*** End of file ***/
