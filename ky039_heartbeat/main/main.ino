/**
 * @file    main.ino
 * @author  Filippo Graziani
 * @date    29/02/2024
 * @brief   A sample code for the heartbeat sensor (KY039)
 * Connections:
 * Sensor       -----       Arduino Uno
 * S                        A0
 * +                        5V
 * -                        GND
 *
 * I used the following program as a starting point:
 * https://projecthub.arduino.cc/Johan_Ha/from-ky-039-to-heart-rate-8c660b
 */

#include "Arduino.h"
#include <stdint.h>

#define     PIN_READ        (A0)

// Sampling with a time of 20ms mean we're sampling at 50Hz, hence we are
// sampling filtering the noise of the electrict light frequency.
//
#define     MS_OF_READ      (20)

// Number of samples to get a read sample.
//
#define     SAMPLE_SIZE     (4)

// The last 4 average and clean readings must be rising.
//
#define     THRESHOLD_RISE  (4)

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

    pinMode(PIN_READ, INPUT);
}   /* setup() */

/**
 * @brief   Loop function.
 * @par     Description
 * Prints the analog read of the flame sensor every 500 milliseconds.
 * It tests also the presence of the alarm.
 * @return  Nothing.
 */
void
loop ()
{
    uint32_t prev_time(millis());
    static int32_t sum(0);
    static uint32_t n_samples(0);
    static double sample_read[SAMPLE_SIZE]{0};
    static uint32_t idx(0);
    static int32_t avg_clean(0);
    static int32_t last_avg_clean(0);
    static uint32_t rise_count(0);
    static bool b_is_rising(false);
    static uint32_t diff_time_ms[3]{0};
    static uint32_t last_beat_ms(0);

    // 20 ms of readings.
    //
    while ((millis() - prev_time) < MS_OF_READ)
    {
        sum += analogRead(PIN_READ);
        ++n_samples;
    }

    // Get an average reading.
    //
    double avg = (double) sum / (double) n_samples;

    sample_read[idx % (SAMPLE_SIZE - 1)] = avg;
    ++idx;
    sum = 0;
    n_samples = 0;

    // Get a value which is an average value between the last 4 readings.
    //
    for (uint32_t scan(0);
         scan < (sizeof(sample_read) / sizeof(sample_read[0])); ++scan)
    {
        avg_clean += sample_read[scan];
    }

    avg_clean /= SAMPLE_SIZE;

    if (avg_clean > last_avg_clean)
    {
        ++rise_count;

        if ((false == b_is_rising) && (rise_count > THRESHOLD_RISE))
        {
            b_is_rising = true;

            diff_time_ms[0] = millis() - last_beat_ms;
            last_beat_ms = millis();

            // A minute (in milliseconds) divided by a weighed average according
            // to the last three beats.
            //
            double beat_freq = 60000.0 / (0.4 * (double) diff_time_ms[0] +
                                          0.3 * (double) diff_time_ms[1] +
                                          0.3 * (double) diff_time_ms[2]);

            diff_time_ms[2] = diff_time_ms[1];
            diff_time_ms[1] = diff_time_ms[0];

            Serial.println(beat_freq);
        }
    }
    else
    {
        rise_count = 0;
        b_is_rising = false;
    }

    last_avg_clean = avg_clean;
}   /* loop() */


/*** End of file ***/

