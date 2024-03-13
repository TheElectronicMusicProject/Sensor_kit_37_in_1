/**
 * @file    main.ino
 * @author  Filippo Graziani
 * @date    13/03/2024
 * @brief   A sample code for the reed switch (KY025)
 * Connections:
 * Sensor       -----       Arduino Uno
 * A0                       A0
 * G                        GND
 * +                        5V
 * D0                       D3
 */

#include "Arduino.h"
#include <stdint.h>

#define     PIN_DIGITAL             (3)
#define     PIN_ANALOG              (A0)
#define     PIN_LED_PWM             (10)

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

    pinMode(PIN_LED_PWM, OUTPUT);
    pinMode(PIN_ANALOG, INPUT);
    pinMode(PIN_DIGITAL, INPUT);
    attachInterrupt(digitalPinToInterrupt(PIN_DIGITAL), irq_detect, RISING);
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
    int32_t led_intesity(analogRead(PIN_ANALOG));

    analogWrite(PIN_LED_PWM, led_intesity / 4);

    if ((true == gb_magnet_detect) && (0 == debounce_time_ms))
    {
        Serial.println("Magnet detected");

        debounce_time_ms = millis();
    }

    Serial.println(led_intesity);

    if ((0 != debounce_time_ms) && ((millis() - debounce_time_ms) > 100))
    {
        debounce_time_ms = 0;
        gb_magnet_detect = false;
        attachInterrupt(digitalPinToInterrupt(PIN_DIGITAL), irq_detect,
                        RISING);
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
