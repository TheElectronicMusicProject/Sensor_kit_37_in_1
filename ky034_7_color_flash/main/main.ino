/**
 * @file    main.ino
 * @author  Filippo Graziani
 * @date    03/03/2024
 * @brief   A sample code for the 7 color flash LED (KY034)
 * Connections:
 * Sensor       -----       Arduino Uno
 * +                        5V
 * [no label]               GND
 * -                        GND
 * @note    A nice but boring device, because it uses an internal chip which
 *          does all the work.
 */

#include "Arduino.h"

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
}   /* setup() */

/**
 * @brief   Loop function.
 * @par     Description
 * Prints a boring message.
 * @return  Nothing.
 */
void
loop ()
{
    Serial.println("Nothing to do...");

    delay(1000);
}   /* loop() */


/*** End of file ***/
