/**
 * @file    main.ino
 * @author  Filippo Graziani
 * @date    10/03/2024
 * @brief   A sample code for the tracking receiver (KY033)
 * Connections:
 * Sensor       -----       Arduino Uno
 * G                        GND
 * V+                       5V
 * S                        D3
 *
 * @note    It detects light colors, creating a falling edge.
 */

#include "Arduino.h"
#include <stdint.h>

void irq_obstacle();

#define     PIN_BTN                 (3)
#define     PIN_LED                 (LED_BUILTIN)

static volatile bool gb_obstacle_detect(false);

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
    pinMode(PIN_BTN, INPUT);
    attachInterrupt(digitalPinToInterrupt(PIN_BTN), irq_obstacle, FALLING);

}   /* setup() */

/**
 * @brief   Loop function.
 * @par     Description
 * Light color obstacles are detected via interrupt.
 * @return  Nothing.
 */
void
loop ()
{
    static uint32_t debounce_time_ms(0);

    if ((true == gb_obstacle_detect) && (0 == debounce_time_ms))
    {
        Serial.println("Obstacle detected");

        digitalWrite(PIN_LED, 1);
        debounce_time_ms = millis();
    }

    if ((0 != debounce_time_ms) && ((millis() - debounce_time_ms) > 500))
    {
        debounce_time_ms = 0;
        gb_obstacle_detect = false;
        digitalWrite(PIN_LED, 0);
        attachInterrupt(digitalPinToInterrupt(PIN_BTN),
                                              irq_obstacle,
                                              FALLING);
    }
}   /* loop() */

/**
 * @brief   Interrupt function.
 * @par     Description
 * Detection of an obstacle with light colors.
 * @return  Nothing.
 */
void
irq_obstacle ()
{
    gb_obstacle_detect = true;
}   /* irq_obstacle() */


/*** End of file ***/
