/**
 * @file    main.ino
 * @author  Filippo Graziani
 * @date    06/03/2024
 * @brief   A sample code for the metal touch sensor (KY036)
 * Connections:
 * Sensor       -----       Arduino Uno
 * A0                       A0
 * G                        GND
 * +                        5V
 * D0                       D3
 *
 * @note    Mantaining the touch pressed, a sort of periodic signal is created.
 *          With the debounce enabled, a clean touch can be detected.
 */

#include "Arduino.h"
#include <stdint.h>

void irq_touch_detect();

#define     PIN_ANALOG_TOUCH    (A0)
#define     PIN_DIGITAL_TOUCH   (3)
#define     PIN_LED             (LED_BUILTIN)

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

    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, 0);
    pinMode(PIN_ANALOG_TOUCH, INPUT);
    pinMode(PIN_DIGITAL_TOUCH, INPUT);
    attachInterrupt(digitalPinToInterrupt(PIN_DIGITAL_TOUCH),
                    irq_touch_detect, FALLING);

}   /* setup() */

/**
 * @brief   Loop function.
 * @par     Description
 * The program sets or unsets the LED when a touch signal higher than a
 * threshold is detected.
 * The threshold is physically managed with a trimmer connected to the sensor.
 * The analog value is printed on the Serial Monitor.
 * @return  Nothing.
 */
void
loop ()
{
    static uint32_t debounce_time_ms(0);
    static uint8_t led_status(0);
    int32_t value(analogRead(PIN_ANALOG_TOUCH));

    if ((true == b_detected) && (0 == debounce_time_ms))
    {
        Serial.println("Sound detected");
        digitalWrite(PIN_LED, led_status);
        led_status = ~led_status;
        debounce_time_ms = millis();
    }

    //Serial.print("Analog Value: ");
    Serial.println(value);

    if ((0 != debounce_time_ms) && ((millis() - debounce_time_ms) > 200))
    {
        debounce_time_ms = 0;
        b_detected = false;
        attachInterrupt(digitalPinToInterrupt(PIN_DIGITAL_TOUCH),
                        irq_touch_detect, FALLING);
    }
}   /* loop() */

/**
 * @brief   Interrupt function.
 * @par     Description
 * Detection of a touch action.
 * @return  Nothing.
 */
void
irq_touch_detect ()
{
    b_detected = true;
}   /* irq_touch_detect() */


/*** End of file ***/
