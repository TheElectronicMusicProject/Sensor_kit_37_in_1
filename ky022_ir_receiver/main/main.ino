/**
 * @file    main.ino
 * @author  Filippo Graziani
 * @date    10/03/2024
 * @brief   A sample code for the infrared receiver (KY022 - VS1838)
 * Connections:
 * Sensor       -----       Arduino Uno
 * -                        GND
 * +                        5V
 * S                        D3
 *
 * @note    To use this sketch, the ky005_ir_emission has been modified, adding
 *          the support to the receiver.
 */

#include "Arduino.h"
#include <stdint.h>

#define     PIN_RX                  (3)
#define     PIN_TX                  (5)
#define     PIN_LED                 (LED_BUILTIN)
#define     ONOFF_TX_US             (562)
#define     TIME_TX_US              (26)

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
    pinMode(PIN_TX, OUTPUT);
    pinMode(PIN_RX, INPUT);

    randomSeed(analogRead(A3));
}   /* setup() */

/**
 * @brief   Loop function.
 * @par     Description
 * An IR LED is used to generate pulses (with PWM) and the IR receiver must be
 * placed in front of it to read the light pulses.
 * No message is transmitted, but the built-in LED is driven.
 * @return  Nothing.
 */
void
loop ()
{
    static uint8_t led_status(0);
    static uint32_t prev_time(micros());
    static uint32_t prev_on_off_time(prev_time);
    static bool b_switch(false);
    int32_t read_tx(0);
    static int32_t prev_read_tx(0);

    // To send logic zeros, we send a PWM series of pulses every 562us.
    //
    if ((micros() - prev_on_off_time) > ONOFF_TX_US)
    {
        b_switch = !b_switch;

        prev_on_off_time = micros();
    }

    // When sending, the PWM pulses are at a frequency of 38kHz.
    //
    if (((micros() - prev_time) > TIME_TX_US) && (true == b_switch))
    {
        analogWrite(PIN_TX, led_status);
        led_status = ~led_status;
        prev_time = micros();
    }

   // Serial.println("RX status is " + String(digitalRead(PIN_RX)));

    read_tx = digitalRead(PIN_RX);

    if (read_tx != prev_read_tx)
    {
        if (0 == read_tx)
        {
            Serial.println("Disabled status");
            digitalWrite(PIN_LED, 0);
        }
        else
        {
            Serial.println("Enabled status");
            digitalWrite(PIN_LED, 1);
        }

        prev_read_tx = read_tx;
    }
}   /* loop() */


/*** End of file ***/
