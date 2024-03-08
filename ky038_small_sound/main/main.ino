/**
 * @file    main.ino
 * @author  Filippo Graziani
 * @date    08/03/2024
 * @brief   A sample code for the small sound microphone sensor (KY038)
 * Connections:
 * Sensor       -----       Arduino Uno
 * A0                       A0
 * G                        GND
 * +                        5V
 * D0                       D3
 */

#include "Arduino.h"
#include <stdint.h>

void irq_sound_detect();

#define     PIN_ANALOG_MICROPHONE   (A0)
#define     PIN_DIGITAL_MICROPHONE  (3)
#define     PIN_LED_PWM             (10)
#define     PIN_LED_SWITCH          (LED_BUILTIN)

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

    pinMode(PIN_LED_PWM, OUTPUT);
    pinMode(PIN_LED_SWITCH, OUTPUT);
    pinMode(PIN_ANALOG_MICROPHONE, INPUT);
    pinMode(PIN_DIGITAL_MICROPHONE, INPUT);
    attachInterrupt(digitalPinToInterrupt(PIN_DIGITAL_MICROPHONE),
                    irq_sound_detect, FALLING);

}   /* setup() */

/**
 * @brief   Loop function.
 * @par     Description
 * The program sets or unsets the LED when a signal higher than a threshold is
 * detected.
 * Furthermore, another LED is driven with a PWM signal.
 * The threshold is physically managed with a trimmer connected to the sensor.
 * The analog value is printed on the Serial Monitor.
 * @return  Nothing.
 */
void
loop ()
{
    static uint32_t debounce_time_ms(0);
    static uint8_t led_status(0);

    // analogWrite goes from 0 to 1023.
    //
    int32_t value(analogRead(PIN_ANALOG_MICROPHONE));

    if ((true == b_detected) && (0 == debounce_time_ms))
    {
        Serial.println("Sound detected");

        digitalWrite(PIN_LED_SWITCH, led_status);
        led_status = ~led_status;
        debounce_time_ms = millis();
    }

    //Serial.print("Analog Value: ");
    Serial.println(value);

    // analogWrite goes from 0 to 255.
    //
    analogWrite(PIN_LED_PWM, value);

    if ((0 != debounce_time_ms) && ((millis() - debounce_time_ms) > 2000))
    {
        debounce_time_ms = 0;
        b_detected = false;
        attachInterrupt(digitalPinToInterrupt(PIN_DIGITAL_MICROPHONE),
                                              irq_sound_detect,
                                              FALLING);
    }
}   /* loop() */

/**
 * @brief   Interrupt function.
 * @par     Description
 * Detection of a high sound.
 * @return  Nothing.
 */
void
irq_sound_detect ()
{
    b_detected = true;
}   /* irq_sound_detect() */


/*** End of file ***/
