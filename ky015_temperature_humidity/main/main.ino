/**
 * @file    main.ino
 * @author  Filippo Graziani
 * @date    14/03/2024
 * @brief   A sample code for the temperature/humidity sensor (KY015 - DHT11)
 * Connections:
 * Sensor       -----       Arduino Uno
 * S                        D3
 * +                        5V
 * -                        GND
 */

#include "Arduino.h"
#include <stdint.h>

#define     PIN_DH11                (3)

namespace dht
{
    class dht11
    {
        private:
            const uint32_t n_byte;
            uint8_t pin;

            /**
            * @brief    Check the level of the DHT11 pin
            * @par      Description
            * Function to check how much time the pin remains high or low.
            * It calculates the time in microseconds before the transitions.
            * It uses a user-defined timeout to avoid blocking state.
            * @param[in]    us_timeout  Timeout, in microseconds.
            * @param[in]    state       Pin status to be checked, if 1 then
            *                           the function returns when it goes 0 and
            *                           viceversa.
            * @return   The time in microseconds between the enter in the
            *           funtion and the transition, -1 in case of timeout.
            */
            int32_t
            dht11_get_level (int32_t us_timeout,
                             uint8_t state)
            {
                int32_t prev_us_time(micros());
                int32_t curr_diff_time(0);

                do
                {
                    curr_diff_time = micros() - prev_us_time;

                    if (curr_diff_time >= us_timeout)
                    {
                        return -1;
                    }
                } while (state == digitalRead(pin));

                return (curr_diff_time);
            }   /* dht11_get_level() */

        public:
            /**
            * @brief    Constructor for the class dht11.
            * @par      Description
            * Initialization of the DHT11 pin on the Arduino Uno side.
            * @param[in]   pin_number   Number of the pin where the DHT11 is
            *                           connected to.
            * @return   Always nothing.
            * @note     Default pin number is D2.
            */
            explicit
            dht11 (uint8_t pin_number = 2) : n_byte(5)
            {
                pin = pin_number;
                pinMode(pin, OUTPUT);
            }   /* dht11() */

            // Default copy constructor.
            //
            //dht11(const dht11& other_dht11);

            // Default destructor.
            //
            //~dht11();

            /**
            * @brief    Constructor for the class dht11.
            * @par      Description
            * Initialization of the DHT11 pin on the Arduino Uno side.
            * @param[in]   pin_number   Number of the pin where the DHT11 is
            *                           connected to.
            * @return   Nothing.
            */
            void
            set_pin (uint8_t pin_number)
            {
                pin = pin_number;
                pinMode(pin, OUTPUT);
            }   /* set_pin() */

            /**
            * @brief    Read temperature and humidity
            * @par      Description
            * Communication with the DHT11 to read the temperature and humidity
            * values.
            * @param[out]   temperature Temperature read, in celsius.
            * @param[out]   humidity    Humidity read, in percentage.
            * @return   0 in case of valid readings, -1 in case of timeout in
            *           the communication, -2 in case of invalid CRC.
            */
            int32_t
            read (float &temperature,
                  float &humidity)
            {
                uint8_t dht_data[n_byte]{0};
                int32_t us_time(0);
                uint32_t idx_byte(0);
                uint32_t idx_bit(0);

                // Start signal from MCU
                //
                pinMode(pin, OUTPUT);
                digitalWrite(pin, LOW);

                delay(20);

                // Pull-up from MCU
                //
                digitalWrite(pin, HIGH);
                delayMicroseconds(40);

                // Set to read the response
                //
                pinMode(pin, INPUT);

                // Wait until the signal goes low (DHT11 starts th
                // communication)
                //
                us_time = dht11_get_level(1000, HIGH);

                if (-1 == us_time)
                {
                    return -1;
                }

                // DHT11 pulls-low
                //
                us_time = dht11_get_level(85, LOW);

                if (-1 == us_time)
                {
                    return -1;
                }

                // DHT11 pulls-up
                //
                us_time = dht11_get_level(85, HIGH);

                if (-1 == us_time)
                {
                    return -1;
                }

                for (idx_byte = 0; idx_byte < 5; ++idx_byte)
                {
                    for (idx_bit = 0; idx_bit < 8; ++idx_bit)
                    {
                        // Start signal from DHT11
                        //
                        us_time = dht11_get_level(50, LOW);

                        if (-1 == us_time)
                        {
                            return -1;
                        }

                        // Bit of information:
                        // - bit 0 if the duration is between 26/28us
                        // - bit 1 if the duration is 70us
                        //
                        us_time = dht11_get_level(75, HIGH);

                        if (-1 == us_time)
                        {
                            return -1;
                        }

                        // 1 has been detected.
                        //
                        if (us_time > 30)
                        {
                            dht_data[idx_byte] |= (1 << (7 - idx_bit));
                        }
                    }
                }

                // First byte is the integer part, second byte the decimal part.
                // Firstly we read the integer 8 bit part, then we shift 8
                // positions to the left (with a multiplication). After we add
                // the decimal 8 bit part.
                // Dividing by 256 (like a shift of 8 positions to the right) we
                // adjust the floating point value.
                //
                humidity = ((dht_data[0] * 256.0) + dht_data[1]) / 256.0;

                temperature = ((dht_data[2] * 256.0) + dht_data[3]) / 256.0;

                if (dht_data[4] == ((dht_data[0] + dht_data[1] + dht_data[2] +
                                     dht_data[3]) & 0xFF))
                {
                    return 0;
                }
                else
                {
                    return -2;
                }
            }   /* dht11_read() */
    };
};

dht::dht11 g_dht11_sensor(PIN_DH11);

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
 * Reading and printing the temperature and the humidity.
 * @return  Nothing.
 */
void
loop ()
{
    float temp(0.0);
    float humid(0.0);
    int32_t ret(0);

    g_dht11_sensor.read(temp, humid);

    if (0 == ret)
    {
        Serial.println("Temperature = " + String(temp) + ", humidity = " +
                       String(humid));
    }
    else
    {
        Serial.println("ret is " + String(ret));
    }

    delay(1000);
}   /* loop() */


/*** End of file ***/
