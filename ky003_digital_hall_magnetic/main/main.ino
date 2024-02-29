/**
 * @file    main.ino
 * @author  Filippo Graziani
 * @date    29/02/2024
 * @brief   A sample code for the digital hall sensor (KY003 - A3144)
 * Connections:
 * Sensor       -----       Arduino Uno
 * +                        GND
 * -                        5V
 * S                        D3
 */

#include "Arduino.h"
#include <stdint.h>

void irq_hall_detect();

#define     PIN_DIGITAL_HALL    (3)

static volatile bool b_detected(false);

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

    pinMode(PIN_DIGITAL_HALL, INPUT);
    attachInterrupt(digitalPinToInterrupt(PIN_DIGITAL_HALL), irq_hall_detect,
                                          FALLING);

}   /* setup() */

/**
 * @brief   Loop function.
 * @par     Description
 * A message is printed on the screen when a south pole magnetic field is
 * detected by the hall sensor.
 * @return  Nothing.
 */
void
loop ()
{
    static uint32_t debounce_time_ms(0);

    if ((true == b_detected) && (0 == debounce_time_ms))
    {
        Serial.println("Magnetic field detected!");
        debounce_time_ms = millis();
    }

    if ((0 != debounce_time_ms) && ((millis() - debounce_time_ms) > 500))
    {
        debounce_time_ms = 0;
        b_detected = false;
        attachInterrupt(digitalPinToInterrupt(PIN_DIGITAL_HALL),
                                              irq_hall_detect,
                                              FALLING);
    }
}   /* loop() */

/**
 * @brief   Interrupt function.
 * @par     Description
 * Detection of a magnetic field (south pole).
 * @return  Nothing.
 */
void
irq_hall_detect ()
{
    b_detected = true;
}   /* irq_hall_detect() */


/*** End of file ***/
