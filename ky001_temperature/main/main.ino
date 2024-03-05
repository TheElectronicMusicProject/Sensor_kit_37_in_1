/**
 * @file    main.ino
 * @author  Filippo Graziani
 * @date    03/03/2024
 * @brief   A sample code for the temperature sensor (KY001 - DS18B20)
 * Connections:
 * Sensor       -----       Arduino Uno
 * S                        D3
 * +                        5V
 * -                        GND
 *
 * It is a programmable resolution 1-Wire digital thermometer.
 *
 * @attention   The library "OneWire" is needed, please refer to the follwing
 *              link: https://www.pjrc.com/teensy/td_libs_OneWire.html
 */

#include "Arduino.h"
#include <OneWire.h>
#include <stdint.h>

#define     PIN_THERMOMETER     (3)
#define     FAMILY_DS18B20_CODE (0x28)

// The class that manages the 1-wire protocol.
//
OneWire g_temperature_sens(PIN_THERMOMETER);

static float read_ds18b20(OneWire& sensor, uint8_t addr[8]);

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
 * Configuration and reading of the thermometer.
 * @return  Nothing.
 */
void
loop ()
{
    uint8_t address[8]{0};
    bool b_device_found(false);

    do
    {
        static uint32_t cnt(0);

        b_device_found = g_temperature_sens.search(address);

        if (true == b_device_found)
        {
            ++cnt;
            Serial.print("Found " + String(cnt) + " devices with address ");

            for (uint32_t idx(0); idx < (sizeof(address) / sizeof(address[0]));
                 ++idx)
            {
                Serial.print(address[idx], HEX);
            }

            Serial.println();
        }
    } while (true == b_device_found);

    if (address[7] != g_temperature_sens.crc8(address, 7))
    {
        Serial.println("Invalid CRC!");
    }

    if (FAMILY_DS18B20_CODE != address[0])
    {
        Serial.println("Invalid Device!");
    }

    for (;;)
    {
        float temperature(read_ds18b20(g_temperature_sens, address));

        Serial.println("Temperature: " + String(temperature) + " degrees");
        delay(1000);
    }
}   /* loop() */

/**
 * @brief   Reads the temperature.
 * @par     Description
 * The function reads the raw data from the sensor, then it converts the data
 * into a valid format (celsius degrees).
 * @param[in]   sensor  The class that manages the 1-wire protocol.
 * @param[in]   addr    Pointer to the array containing the address
 * @return  The temperature, in celsius degrees.
*/
static float
read_ds18b20 (OneWire& sensor, uint8_t addr[8])
{
    float temp(0);
    uint8_t data[9]{0};

    // Start conversion, without the use of the parasitic device.
    //
    uint8_t b_is_present(sensor.reset());
    sensor.select(addr);
    sensor.write(0x44, 0);

    // If using 12 bit conversion, the maximum conversion time is 750ms.
    //
    delay(750);

    // Read scratchpad, so temperature LSB, temperature MSB, TH, TL, CONFIG and
    // CRC.
    //
    b_is_present = sensor.reset();
    sensor.select(addr);
    sensor.write(0xBE, 0);

    if (1 == b_is_present)
    {
        for (uint32_t idx(0); idx < (sizeof(data) / sizeof(data[0])); ++idx)
        {
            data[idx] = sensor.read();
            Serial.print(data[idx], HEX);
        }

        Serial.println();
    }

    if (data[8] != sensor.crc8(data, 8))
    {
        Serial.println("Invalid CRC!");
    }
    else
    {
        // Get raw reading.
        //
        int16_t raw = data[1];
        raw <<= 8;
        raw |= data[0];

        // Get resolution bits.
        //
        int8_t conf(data[4] & ((1 << 5) | (1 << 6)));

        // Get rid of the low bits, which are undefined.
        //
        switch (conf >> 5)
        {
            // 9 bit resolution
            //
            case 0x00:
                conf &= ~0x7;
            break;

            // 10 bit resolution
            //
            case 0x01:
                conf &= ~0x3;
            break;

            // 11 bit resolution
            //
            case 0x02:
                conf &= ~0x1;
            break;

            // 12 bit resolution
            //
            case 0x03:

            break;

            default:
                Serial.println("ERROR");
            break;
        }
        
        // Divide the result by 16, which is like shifting to the right 16
        // positions.
        // That's because:
        // 
        // LSB:
        // 2^3 - 2^2 - 2^1 - 2^0 - 2^-1 - 2^-2 - 2^-3 - 2^-4
        // MSB:
        // S - S - S - S - S - 2^6 - 2^5 - 2^4
        //
        temp = (float) raw / 16.0;
    }

    return temp;
}   /* read_ds18B20() */

/*** End of file ***/
