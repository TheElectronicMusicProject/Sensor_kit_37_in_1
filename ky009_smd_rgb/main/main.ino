/**
 * @file    main.ino
 * @author  Filippo Graziani
 * @date    03/03/2024
 * @brief   A sample code for the SMD RGB LED (KY009)
 * Connections:
 * Sensor       -----       Arduino Uno
 * B                        9
 * R                        11
 * G                        6
 * -                        GND
 @attention This LED needs a resistor for each single LED.
 */

#include "Arduino.h"
#include <stdint.h>

class rgb_led
{
    private:
        uint8_t red;
        uint8_t green;
        uint8_t blue;
    public:
        /**
        * @brief   Constructor for the class rgb_led.
        * @par     Description
        * Initialization of the RGB LED pins on the Arduino Uno side.
        * @param[in]   red_pin     The pin where the red led is connected to.
        * @param[in]   green_pin   The pin where the green led is connected to.
        * @param[in]   blue_pin    The pin where the blue led is connected to.
        * @return  Always nothing.
        */
        explicit
        rgb_led (uint8_t red_pin,
                 uint8_t green_pin,
                 uint8_t blue_pin)
        {
            red = red_pin;
            green = green_pin;
            blue = blue_pin;

            pinMode(red, OUTPUT);
            pinMode(green, OUTPUT);
            pinMode(blue, OUTPUT);
        }   /* rgb_led() */

        // Default copy constructor.
        //
        //rgb_led(const rgb_led& other_rgb_led);

        // Default destructor.
        //
        //~rgb_led();

        /**
        * @brief   Write a PWM value on the three leds.
        * @par     Description
        * The given analog values are written on the Arduino Uno pins.
        * @param[in]   red_val     The value for the red led.
        * @param[in]   green_val   The value for the green led.
        * @param[in]   blue_val    The value for the blue led.
        * @return  Nothing.
        */
        void
        write (uint8_t red_val,
               uint8_t green_val,
               uint8_t blue_val)
        {
            analogWrite(red, red_val);
            analogWrite(green, green_val);
            analogWrite(blue, blue_val);
        }   /* write() */
};

#define     PIN_RED     (11)
#define     PIN_GREEN   (6)
#define     PIN_BLUE    (9)

// The class which manages the LED status.
//
static rgb_led my_rgb_led(PIN_RED, PIN_GREEN, PIN_BLUE);

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
}   /* setup() */

/**
 * @brief   Loop function.
 * @par     Description
 * Tests the fading of the colors of the RGB LED.
 * @return  Nothing.
 */
void
loop ()
{
    Serial.println("Transition blue to green");

    // Blue to green.
    //
    for (uint8_t level(0); level < 255; ++level)
    {
        my_rgb_led.write(0, level, 255 - level);
        delay(10);
    }

    Serial.println("Transition green to red");

    // Green to red.
    //
    for (uint8_t level(0); level < 255; ++level)
    {
        my_rgb_led.write(level, 255 - level, 0);
        delay(10);
    }

    Serial.println("Transition red to blue");

    // Red to blue.
    //
    for (uint8_t level(0); level < 255; ++level)
    {
        my_rgb_led.write(255 - level, 0, level);
        delay(10);
    }
}   /* loop() */


/*** End of file ***/
