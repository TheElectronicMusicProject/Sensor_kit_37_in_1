/**
 * @file    main.ino
 * @author  Filippo Graziani
 * @date    13/03/2024
 * @brief   A sample code for the analog hall sensor (KY035 - AH49E)
 * Connections:
 * Sensor       -----       Arduino Uno
 * -                        GND
 * +                        5V
 * S                        A0
 */

#include "Arduino.h"
#include <stdint.h>

#define     PIN_ANALOG_HALL     (A0)
#define     PIN_LED_PWM         (10)

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
    pinMode(PIN_LED_PWM, OUTPUT);
}   /* setup() */

/**
 * @brief   Loop function.
 * @par     Description
 * The analog read is used to drive an LED through PWM.
 * @return  Nothing.
 */
void
loop ()
{
    int32_t val(analogRead(PIN_ANALOG_HALL));
    int32_t pwm(map(val, 0, 1023, 0, 255));
    
    analogWrite(PIN_LED_PWM, pwm);

    Serial.println(val);    
}   /* loop() */


/*** End of file ***/
