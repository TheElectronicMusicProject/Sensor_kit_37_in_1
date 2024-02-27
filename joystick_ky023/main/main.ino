/**
 * @file    main.ino
 * @author  Filippo Graziani
 * @date    27/02/2024
 * @brief   A sample code for the analog joystick (KY023)
 * Connections:
 * Joystick     -----       Arduino Uno
 * GND                      GND
 * +5V                      5V
 * VRX                      A0
 * VRY                      A1
 * SW                       2
 */

#include "Arduino.h"
#include <stdint.h>

static void irq_joystick_sw();

#define     PIN_VRX     (A0)
#define     PIN_VRY     (A1)
#define     PIN_SW      (2)

// Switch status.
//
static volatile bool b_sw_clicked(false);

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

    pinMode(PIN_VRX, INPUT);
    pinMode(PIN_VRY, INPUT);
    pinMode(PIN_SW, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_SW), irq_joystick_sw, FALLING);
}   /* setup() */

/**
 * @brief   Loop function.
 * @par     Description
 * Print of the analog read of the joystick every 500 milliseconds.
 * @return  Nothing.
 */
void
loop ()
{
    // The previous print.
    //
    static uint32_t time_prev(0);
    static uint32_t debounce_time(0);

    if ((millis() - time_prev) > 500)
    {
        time_prev = millis();

        int32_t x_axis = analogRead(PIN_VRX);
        int32_t y_axis = analogRead(PIN_VRY);
        int32_t switch_stat = digitalRead(PIN_SW);

        if (x_axis > 530)
        {
            Serial.print("Going forwards ");
        }
        else if (x_axis < 500)
        {
            Serial.print("Going backwards ");
        }
        else
        {
            Serial.print("Stationary along the vertical direction ");
        }

        if (y_axis > 530)
        {
            Serial.print("and turning right ");
        }
        else if (y_axis < 500)
        {
            Serial.print("and turning left ");
        }
        else
        {
            Serial.print("and stationary along the horizontal direction ");
        }

        Serial.println("\n---\n");

        Serial.println("X axis: " + String(x_axis) + " "
                       "Y axis: " + String(y_axis) + " "
                       "Switch: " + String(switch_stat));
    }

    if (true == b_sw_clicked)
    {
        Serial.println("Switch has been clicked!");
        b_sw_clicked = false;
        detachInterrupt(digitalPinToInterrupt(PIN_SW));
        debounce_time = millis();
    }

    if ((0 != debounce_time) && ((millis() - debounce_time) > 600))
    {
        debounce_time = 0;
        attachInterrupt(digitalPinToInterrupt(PIN_SW), irq_joystick_sw,
                        FALLING);
    }
}   /* loop() */

/**
 * @brief   Interrupt function.
 * @par     Description
 * Returns the switch status
 * @return  Nothing.
 */
static void
irq_joystick_sw ()
{
    b_sw_clicked = true;
}   /* irq_joystick_sw() */


/*** End of file ***/
