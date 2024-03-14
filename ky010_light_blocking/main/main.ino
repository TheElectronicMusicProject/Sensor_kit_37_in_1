/**
 * @file    main.ino
 * @author  Filippo Graziani
 * @date    14/03/2024
 * @brief   A sample code for the light blocking sensor (KY010)
 * Connections:
 * Sensor       -----       Arduino Uno
 * -                        GND
 * [no label]               5V
 * S                        D3
 */

#include "Arduino.h"
#include <stdint.h>

#define     PIN_LIGHT               (3)
#define     PIN_LED                 (LED_BUILTIN)

static void irq_light_block();

static volatile bool gb_detected(false);

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
    pinMode(PIN_LIGHT, INPUT);
    attachInterrupt(digitalPinToInterrupt(PIN_LIGHT), irq_light_block, RISING);
}   /* setup() */

/**
 * @brief   Loop function.
 * @par     Description
 * Checking the presence of the light of the sensor, hence if there is no light
 * then there is an obstacle.
 * @return  Nothing.
 */
void
loop ()
{
    static uint32_t debounce_time_ms(0);

    if ((true == gb_detected) && (0 == debounce_time_ms))
    {
        detachInterrupt(digitalPinToInterrupt(PIN_LIGHT));
        Serial.println("Sound detected");
        digitalWrite(PIN_LED, LOW);
        debounce_time_ms = millis();
    }

    if ((0 != debounce_time_ms) && ((millis() - debounce_time_ms) > 500))
    {
        digitalWrite(PIN_LED, HIGH);
        debounce_time_ms = 0;
        gb_detected = false;
        attachInterrupt(digitalPinToInterrupt(PIN_LIGHT), irq_light_block,
                        RISING);
    }
}   /* loop() */

/**
 * @brief   Interrupt function.
 * @par     Description
 * Detection of a light blocked status.
 * @return  Nothing.
 */
static void
irq_light_block ()
{
    gb_detected = true;
}   /* irq_light_block() */


/*** End of file ***/
