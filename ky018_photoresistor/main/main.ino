/**
 * @file    main.ino
 * @author  Filippo Graziani
 * @date    09/03/2024
 * @brief   A sample code for the photoresistor (KY018)
 * Connections:
 * Sensor       -----       Arduino Uno
 * -                        GND
 * [no label]               5V
 * S                        A0
 */

#include "Arduino.h"
#include <stdint.h>

#define     PIN_LIGHT               (A0)
#define     PIN_LED_PWM             (10)

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
    pinMode(PIN_LIGHT, INPUT);
}   /* setup() */

/**
 * @brief   Loop function.
 * @par     Description
 * Depending on the photoresistor's value (driven by the amount of light), the
 * LED changes its PWM value increasing as the quantity of light decreases.
 * @return  Nothing.
 */
void
loop ()
{
    int32_t light_val(analogRead(PIN_LIGHT));
    int32_t pwm_val(map(light_val, 0, 1023, 0, 255));

    analogWrite(PIN_LED_PWM, pwm_val);
}   /* loop() */


/*** End of file ***/
