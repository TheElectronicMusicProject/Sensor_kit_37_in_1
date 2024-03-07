/**
 * @file    main.ino
 * @author  Filippo Graziani
 * @date    03/03/2024
 * @brief   A sample code for the relay (KY019)
 * Connections:
 * Sensor       -----       Arduino Uno
 * S                        D8
 * +                        5V
 * -                        GND
 * @note    Use the normally open contact of the relay.
 */

#include "Arduino.h"
#include <stdint.h>

#define     PIN_RELAY       (8)


/**
 * @brief   Setup function.
 * @par     Description
 * Setting of the Arduino's pins and the serial port.
 * @return  Nothing.
 */
void
setup ()
{
    pinMode(PIN_RELAY, OUTPUT);
}   /* setup() */

/**
 * @brief   Loop function.
 * @par     Description
 * An easy control of a digital pin of Arduino.
 * You can connect whatever you want to the other side of the relay (e.g. I
 * used a little DC motor).
 * The relay is switched every 1 second.
 * @return  Nothing.
 */
void
loop ()
{
    static uint32_t prev_time(millis());
    static bool b_relay_is_on(false);

    if ((millis() - prev_time) > 1000)
    {
        digitalWrite(PIN_RELAY, b_relay_is_on);
        b_relay_is_on = !b_relay_is_on;
        prev_time = millis();
    }
}   /* loop() */


/*** End of file ***/
