/**
 * @file    main.ino
 * @author  Filippo Graziani
 * @date    13/03/2024
 * @brief   A sample code for the shock sensor (KY002)
 * Connections:
 * Sensor       -----       Arduino Uno
 * -                        GND
 * [no label]               5V
 * S                        D3
 */

#include "Arduino.h"
#include <stdint.h>

#define     PIN_SHOCK               (3)
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
    pinMode(PIN_SHOCK, INPUT);
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
    if (LOW == digitalRead(PIN_SHOCK))
    {
        digitalWrite(PIN_LED, HIGH);
        Serial.println("Vibration detected");
    }
    else
    {
        digitalWrite(PIN_LED, LOW);
    }
}   /* loop() */


/*** End of file ***/
