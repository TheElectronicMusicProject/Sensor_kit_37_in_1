/**
 * @file    main.ino
 * @author  Filippo Graziani
 * @date    03/03/2024
 * @brief   A sample code for the linear hall sensor (KY024)
 * Connections:
 * Sensor       -----       Arduino Uno
 * A0                       A0
 * G                        GND
 * +                        5V
 * D0                       D3
 */

#include "Arduino.h"
#include <stdint.h>

void irq_hall_detect();

#define     PIN_ANALOG_HALL     (A0)
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

    pinMode(PIN_ANALOG_HALL, INPUT);
    pinMode(PIN_DIGITAL_HALL, INPUT);
    attachInterrupt(digitalPinToInterrupt(PIN_DIGITAL_HALL), irq_hall_detect,
                                          FALLING);

}   /* setup() */

/**
 * @brief   Loop function.
 * @par     Description
 * A message is printed on the screen when a south pole magnetic field is
 * detected by the hall sensor.
 * Furthermore, the analog read of the sensor is displayed on the screen and can
 * be plotted through the Serial Plotter.
 * @return  Nothing.
 */
void
loop ()
{
    static uint32_t debounce_time_ms(0);

    Serial.println(analogRead(PIN_ANALOG_HALL));

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
