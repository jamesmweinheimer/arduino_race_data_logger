/*
 * MLX90621.h
 *
 *  Created on: 08.07.2014
 *      Author: Max Ritter
 *
 *  Adapted by https://github.com/longjos
 *  	Adapted for use with Arduino UNO
 */

#ifndef MLX90621_2_H_
#define MLX90621_2_H_

#ifdef __cplusplus

// Libraries to be included
#include <Arduino.h>
#include <Wire.h>

const byte PACKET_SIZE = 32;

const int EEPROM_SIZE = 256;
const uint8_t START_PIXEL = 16;
const uint8_t NUM_PIXELS = 32;
const byte START_COL = 0;
const byte START_ROW = 1;
const byte NUM_COLS = 16;
const byte NUM_ROWS = 2;

const byte RAM_ADDRESS = 0x60;
const byte EEPROM_ADDRESS = 0x50;
const byte CONFIG_ADDRESS = 0x92;

const byte READ_EEPROM = 0x00;
const byte READ_RAM = 0x02;
const byte WRITE_REGISTER = 0x03;
const byte WRITE_OSC_TRIM = 0x04;

const byte PTAT_ADDRESS = 0x40;
const byte CPIX_ADDRESS = 0x41;
const byte OSC_CHECK = 0xAA;
const byte CONF_CHECK = 0x55;

// Begin registers
const byte CAL_ACOMMON_L = 0xD0;
const byte CAL_ACOMMON_H = 0xD1;
const byte CAL_ACP_L = 0xD3;
const byte CAL_ACP_H = 0xD4;
const byte CAL_BCP = 0xD5;
const byte CAL_alphaCP_L = 0xD6;
const byte CAL_alphaCP_H = 0xD7;
const byte CAL_TGC = 0xD8;
const byte CAL_AI_SCALE = 0xD9;
const byte CAL_BI_SCALE = 0xD9;

// EEPROM addresses for ambient temperature constants
const byte VTH_L = 0xDA;
const byte VTH_H = 0xDB;
const byte KT1_L = 0xDC;
const byte KT1_H = 0xDD;
const byte KT2_L = 0xDE;
const byte KT2_H = 0xDF;
const byte KT_SCALE = 0xD2;

// Common sensitivity coefficients
const byte CAL_A0_L = 0xE0;
const byte CAL_A0_H = 0xE1;
const byte CAL_A0_SCALE = 0xE2;
const byte CAL_DELTA_A_SCALE = 0xE3;
const byte CAL_EMIS_L = 0xE4;
const byte CAL_EMIS_H = 0xE5;
const byte CAL_KSTA_L = 0xE6;
const byte CAL_KSTA_H = 0xE7;

const byte OSC_TRIM_VALUE = 0xF7;
const byte POR_BIT = 10;

class MLX90621 {
   private:
    /* Variables */
    uint8_t eeprom_buffer[EEPROM_SIZE]; /**<RAM buffer to store the sensor's EEPROM data. Contains compensation values*/
    int _refresh_rate;                  /**<Refresh rate of the sensor in Hz. {0 (0.5 Hz), 1, 2, 4, 8, 16, 32}*/

    // Ambient temperature compensation constants
    uint16_t k_t1_scale;
    uint16_t k_t2_scale;
    float v_th;
    float k_t1;
    float k_t2;

    // Object temperature compensation constants
    float resolution_comp;
    float tgc;
    float emissivity;
    int16_t a_common;
    uint16_t a_i_scale;
    uint16_t b_i_scale;
    float alpha_cp;
    float a_cp;
    float b_cp;

    // Object temperature frame constants
    float ambient;
    float tak4;
    float v_cp_off_comp;

    // Config methods

    /**
    * Load the configuration data onto the sensor.
    * This must happen before the sensor is able to give accurate temperature information.
    */
    void load_sensor();

    /**
    * Read in sensor's EEPROM into MCU RAM for fast retrieval
    * The EEPROM contains the compensation factors needed for temperature measurement
    */
    void read_EEPROM();

    /**
    * Write the oscillator trim value from the sensor's EEPROM into the sensor's control registers
    */
    void write_trimming_value();

    /**
    * Set the configuration registers of the sensor
    * Default configuration values used for most of the registers
    */
    void set_configuration();

    /**
    * Calculate the constants needed for temperature calculation
    * This only needs to be performed once as the constants are hard-coded for each sensor.
    */
    void precalculate_constants();

    /**
    * Get the resolution bits of the sensor.
    * {00 = 15-bit,
    * 01 = 16-bit,
    * 10 = 17-bit,
    * 11 = 18-bit}
    * @return Resolution bits of the sensor
    */
    uint8_t get_resolution();

    /**
    * Read the sensor's configuration register.
    * See section 8.2.2.1 of the MLX90621 datasheet for meaning of each bit
    */
    uint16_t get_config();

    /**
    * Reloads the sensor configuration data if the sensor has not been initialised.
    * A power reset or brown-out may cause the configuration data to be lost during operation.
    * This check ensures that the sensor has been configured and is ready for sensor reads.
    */
    void check_configuration();

    /**
    * Check the ready status of the sensor to see if it needs to be reloaded
    * The POR bit in the sensor's configuration is set if the device has been initialised.
    * If the POR bit is clear, then the sensor has restarted and needs to be reloaded
    *
    * @return Status of the sensor. {True: sensor reload required; False: sensor has been initialised}
    */
    bool needs_reload();

    // Utilities
    /**
    * Return the 2's compliment of a 16-bit integer
    * @param highByte MSB of the integer
    * @param lowByte LSB of the integer
    * @return 2's compliment of the input integer
    */
    int16_t twos_16(uint8_t highByte, uint8_t lowByte);

    /**
    * Return the 2's compliment of an 8-bit integer
    * @param byte Byte to transform
    * @return 2's compliment of the input integer
    */
    int8_t twos_8(uint8_t byte);

    /**
    * Combine two bytes into an unsigned word.
    * @param highByte Most significant byte
    * @param lowByte Least significant byte
    * @param Unsigned word of the combined bytes
    */
    uint16_t unsigned_16(uint8_t highByte, uint8_t lowByte);

    // Ambient temperature
    /**
    * Read Proportional to Absolute Temperature sensor to find the ambient temperature of the chip
    * @return Raw PTAT data from the sensor
    */
    int get_PTAT();

    // Object temperature
    /**
    * Calulate the frame constants needed for pixel temperature compensation.
    * Must be run before measuring each frame.
    */
    void precalculate_frame_values();

    /**
    * Get the value of the compensation pixel from the sensor.
    * Not entirely sure what this one does, but the datasheet does lots of maths with it.
    *
    * @return Value of the compensation pixel
    */
    int get_compensation_pixel();

    /**
    * Read in raw IR temperature data from the sensor's RAM
    * The Wire library cannot handle packets larger than 32-bytes, so the registry must be read in chunks.
    *
    * @param ir_buffer Buffer to store the data. Must be at least NUM_PIXELS wide.
    */
    void get_IR(int ir_buffer[]);

    /**
    * Calculate the temperature recorded by the sensor for a given pixel.
    *
    * @param pixel_num Number of the pixel to calculate. [0-63]. First row sits between 0-15; second row is between
    * 16-31, etc.
    * @param ir_data array containing the raw IR data from the sensor
    * @return The recorded temperature of the pixel in deg C.
    */
    float calculate_pixel(uint8_t pixel_num, int ir_data[]);

   public:
    /**
    * Start up the MLX90621 sensor and prepare for reads.
    * @param refresh_rate Refresh rate of the sensor in frames per second. {0 (0.5), 1, 2, 4, 8, 16, 32}
    */
    void initialise(int refresh_rate);

    /**
    * Get the temperatures recorded by the sensor and insert them into the given buffer
    * The buffer must be at least 64 items wide to fit the data.
    *
    * @param output_buffer array to insert the temperature into.
    */
    void get_temperatures(float output_buffer[NUM_PIXELS]);
    void get_temperatures(float output_buffer[NUM_ROWS][NUM_COLS], bool mirror_frame = false);

    /**
    * Print the temperature values of all pixels to the selected serial interface.
    * Temperatures are printed as a comma-separated list.
    * Pixels are ordered by row.
    *
    * @param ser The serial interface to print the temperatures to.
    */
    void print_temperatures(HardwareSerial &ser);

    /**
    * Read the sensor and calculate the ambient temperature.
    * The calculated temperature is stored in a class variable to provide access to other functions
    * @return Ambient temperature measured by the sensor in deg C.
    */
    float get_ambient_temperature();
};

#endif
#endif
