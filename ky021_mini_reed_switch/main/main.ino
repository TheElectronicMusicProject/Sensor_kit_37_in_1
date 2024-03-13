/**
 * @file    main.ino
 * @author  Filippo Graziani
 * @date    13/03/2024
 * @brief   A sample code for the mini reed switch (KY025)
 * Connections:
 * Sensor       -----       Arduino Uno
 * -                        GND
 * [no label]               5V
 * S                        D3
 */

#include "Arduino.h"
#include <stdint.h>

#define     PIN_REED                (3)
#define     PIN_LED                 (LED_BUILTIN)

static void irq_detect();

static bool gb_magnet_detect(false);

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
    pinMode(PIN_REED, INPUT);
    attachInterrupt(digitalPinToInterrupt(PIN_REED), irq_detect, RISING);
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
    static uint32_t debounce_time_ms(0);

    if ((true == gb_magnet_detect) && (0 == debounce_time_ms))
    {
        Serial.println("Magnet detected");

        debounce_time_ms = millis();
        digitalWrite(PIN_LED, HIGH);
    }

    if ((0 != debounce_time_ms) && ((millis() - debounce_time_ms) > 100))
    {
        digitalWrite(PIN_LED, LOW);
        debounce_time_ms = 0;
        gb_magnet_detect = false;
        attachInterrupt(digitalPinToInterrupt(PIN_REED), irq_detect, RISING);
    }
}   /* loop() */

/**
 * @brief   Interrupt function.
 * @par     Description
 * Detection of a magnetic field which closes the switch.
 * @return  Nothing.
 */
static void
irq_detect ()
{
    gb_magnet_detect = true;
}   /* irq_detect() */


/*** End of file ***/
