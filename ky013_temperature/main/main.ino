/**
 * @file    main.ino
 * @author  Filippo Graziani
 * @date    07/03/2024
 * @brief   A sample code for the temperature sensor (KY013)
 * Connections:
 * Sensor       -----       Arduino Uno
 * -                        5V
 * [no label]               GND
 * S                        A0
 *
 * @note    It is a variable NTC resistor.
 * @attention   The pins are wrongly named.
 */

#include "Arduino.h"
#include <stdint.h>
#include <math.h>

#define     PIN_TEMP    (A0)
#define     PIN_LED     (LED_BUILTIN)

class temp_t
{
    private:
        uint8_t sensor;
        const double const1;
        const double const2;
        const double const3;

        /**
        * @brief   Conversion from raw read to kelvin.
        * @par     Description
        * Conversion of the input analog read into the kelvin value.
        * @param[in]   raw_read     The analog read.
        * @return  The converted temperature (in kelvin).
        */
        double
        convert_raw_to_kelvin (int32_t raw_read)
        {
            // Calculates the thermisor resistance by knowing the other
            // resistor's value that is 10kohm, the maximum analog read and
            // the effective read.
            //
            double calc(log(10000.0 * ((1024.0 / (double) raw_read) - 1.0)));
            calc = 1.0 / (const1 + (const2 * calc) +
                          (const3 * calc * calc * calc));

            return calc;
        }   /* convert_raw_to_kelvin() */
    public:
        /**
        * @brief   Constructor for the class temp_t.
        * @par     Description
        * Initialization of the thermistor pin on the Arduino Uno side.
        * @param[in]   temp_pin     The pin where the analog signal of the
        *                           thermometer is connected.
        * @return  Always nothing.
        * @note    By default it uses pin A0.
        */
        explicit
        temp_t (uint8_t temp_pin = A0) : const1(0.001129148),
                                         const2(0.000234125),
                                         const3(0.0000000876741)
        {
            sensor = temp_pin;

            pinMode(sensor, INPUT);
        }   /* temp_t() */

        // Default copy constructor.
        //
        //temp_t(const temp_t& other_temp_t);

        // Default destructor.
        //
        //~temp_t();

        /**
        * @brief   Temperature in kelvin.
        * @par     Description
        * The function reads the kelvin temperature
        * @return  The temperature in kelvin.
        */
        double
        read_kelvin ()
        {
            int32_t temperature(analogRead(sensor));
            
            return (convert_raw_to_kelvin(temperature));
        }   /* read_kelvin() */

        /**
        * @brief   Temperature in celsius.
        * @par     Description
        * The function reads the celsius temperature
        * @return  The temperature in celsius.
        */
        double
        read_celsius ()
        {
            double temp(read_kelvin());

            temp -= 273.15;

            return (temp);
        }   /* read_celsius() */
};

static temp_t g_temp_sens(PIN_TEMP);

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
}   /* setup() */

/**
 * @brief   Loop function.
 * @par     Description
 * The analog temperature is read from the class and it's compared to a
 * threshold to check if the LED must be turned on.
 * @return  Nothing.
 */
void
loop ()
{
    double value(g_temp_sens.read_celsius());

    Serial.println(value);
    
    // Test of a threshold.
    //
    if (value >= 28.0)
    {
        digitalWrite(PIN_LED, 1);
    }
    else
    {
        digitalWrite(PIN_LED, 0);
    }
}   /* loop() */


/*** End of file ***/
