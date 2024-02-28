/**
 * @file    main.ino
 * @author  Filippo Graziani
 * @date    28/02/2024
 * @brief   A sample code for the flame sensor (KY026)
 * Connections:
 * Sensor       -----       Arduino Uno
 * A0                       A0
 * G                        GND
 * +                        5V
 * D0                       2
 */

#include "Arduino.h"
#include <stdint.h>

static void irq_flame_detect();

#define     PIN_ANALOG      (A0)
#define     PIN_THRESHOLD   (2)

// Threshold to show an alarm.
//
static volatile bool b_flame_detected(false);

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

    pinMode(PIN_ANALOG, INPUT);
    pinMode(PIN_THRESHOLD, INPUT);
    attachInterrupt(digitalPinToInterrupt(PIN_THRESHOLD), irq_flame_detect,
                                          RISING);
}   /* setup() */

/**
 * @brief   Loop function.
 * @par     Description
 * Prints the analog read of the flame sensor every 500 milliseconds.
 * It tests also the presence of the alarm.
 * @return  Nothing.
 */
void
loop ()
{
    // The previous print.
    //
    static uint32_t time_prev(0);

    if ((millis() - time_prev) > 500)
    {
        time_prev = millis();

        int32_t value = analogRead(PIN_ANALOG);
        int32_t flame_status = digitalRead(PIN_THRESHOLD);
        
        Serial.println("Value: " + String(value) + " "
                       "Flame detected: " + String(flame_status));
    }

    if (true == b_flame_detected)
    {
        Serial.println("Fire detected!");
        b_flame_detected = false;
    }
}   /* loop() */

/**
 * @brief   Interrupt function.
 * @par     Description
 * Enables the alarm trigger.
 * @return  Nothing.
 */
static void
irq_flame_detect ()
{
    b_flame_detected = true;
}   /* irq_flame_detect() */


/*** End of file ***/
