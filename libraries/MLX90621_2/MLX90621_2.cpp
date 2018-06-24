/*
 * MLX90621_2.cpp
 *
 *  Created on: 18.11.2013
 *      Author: Max
 *
 *  Adapted by https://github.com/longjos
 *  	Adapted for use with Arduino UNO
 *
 * Adapted again by https://github.com/Leenix
 * 		- Removed hard memory allocations for IR data and calculated
 *temperatures
 *		- Added option to print values directly instead of storing them
 *		- Removed unused or redundant variables
 *		- Did a little bit of Spring cleaning
 */
#include "MLX90621_2.h"

///////////////////////////////////////////////////////////////////////////////
// Configuration

void MLX90621::initialise(int refresh_rate) {
  /**
  * Start up the MLX90621 sensor and prepare for reads.
  *
  * @param refresh_rate Refresh rate of the sensor in frames per second. {0
  * (0.5), 1, 2, 4, 8, 16, 32}
  */

  // 7.1 - The initialisation process must start at least 5ms after POR release
  delay(5);
  _refresh_rate = refresh_rate;
  load_sensor();
}

void MLX90621::load_sensor() {
  /**
  * Load the configuration data onto the sensor.
  * This must happen before the sensor is able to give accurate temperature
  * information.
  */
  read_EEPROM();
  write_trimming_value();
  set_configuration();
  precalculate_constants();
}

void MLX90621::read_EEPROM() {
  /**
  * Read in sensor's EEPROM into MCU RAM for fast retrieval
  * The EEPROM contains the compensation factors needed for temperature
  * measurement
  */

  // Read in blocks of 32 bytes to accomodate Wire library
  for (int j = 0; j < EEPROM_SIZE; j += PACKET_SIZE) {
    Wire.beginTransmission(EEPROM_ADDRESS);
    Wire.write(j);
    Wire.endTransmission(false);
    Wire.requestFrom(EEPROM_ADDRESS, PACKET_SIZE);
    for (int i = 0; i < PACKET_SIZE; i++) {
      eeprom_buffer[j + i] = (uint8_t)Wire.read();
    }
  }
}

void MLX90621::write_trimming_value() {
  /**
  * Write the oscillator trim value from the sensor's EEPROM into the sensor's
  * control registers
  */
  byte msbyte = 0x00;

  Wire.beginTransmission(RAM_ADDRESS);
  Wire.write(WRITE_OSC_TRIM);

  // LSB Byte check - LSByte - 0xAA
  Wire.write((byte)eeprom_buffer[OSC_TRIM_VALUE] - OSC_CHECK);

  // LSByte
  Wire.write(eeprom_buffer[OSC_TRIM_VALUE]);

  // MSByte check - 0x00 - 0xAA
  Wire.write(msbyte - OSC_CHECK);
  Wire.write(msbyte);
  Wire.endTransmission();
}

void MLX90621::set_configuration() {
  /**
  * Set the configuration registers of the sensor
  * Default configuration values used for most of the registers
  *
  * 15	0 - Melexis reserved
  * 14	1 - ADC low reference enabled (clearing this bit impacts calibration)
  * 13	0 - Melexis reserved
  * 12	0 - EEPROM enabled
  * 11	0 - I2C FM+ mode enabled
  * 10	1 - POR flag reset (must be written during configuration)
  * 9		0/1 - IR measurement flag (cannot be written)
  * 8		0 - Unused
  * 7		0 - Sleep mode disabled
  * 6		0 - Continuous mode enabled
  * 5		0 - ADC resolution 1 (15-bit)
  * 4		0 - ADC resolution 2 (15-bit)
  * 3		1 - Refresh rate 1 (1 Hz)
  * 2		1 - Refresh rate 2 (1 Hz)
  * 1		1 - Refresh rate 3 (1 Hz)
  * 0		0 - Refresh rate 4 (1 Hz)
  *
  */

  // Find the LSB that needs to be written for the refresh rate; ADC set to
  // 15-bit resolution
  byte Hz_LSB;
  switch (_refresh_rate) {
  case 0: // 0.5 Hz
    Hz_LSB = 0b00111111;
    break;
  case 1:
    Hz_LSB = 0b00111110;
    break;
  case 2:
    Hz_LSB = 0b00111101;
    break;
  case 4:
    Hz_LSB = 0b00111100;
    break;
  case 8:
    Hz_LSB = 0b00111011;
    break;
  case 16:
    Hz_LSB = 0b00111010;
    break;
  case 32:
    Hz_LSB = 0b00111001;
    break;
  default: // 1 Hz
    Hz_LSB = 0b00111110;
  }

  // Default configuration; See 8.2.2 - Internal registers
  byte defaultConfig_H = 0b01000110;

  Wire.beginTransmission(RAM_ADDRESS);
  Wire.write(WRITE_REGISTER);
  Wire.write((byte)Hz_LSB - CONF_CHECK);
  Wire.write(Hz_LSB);
  Wire.write(defaultConfig_H - CONF_CHECK);
  Wire.write(defaultConfig_H);
  Wire.endTransmission();
}

void MLX90621::precalculate_constants() {
  /**
  * Calculate the constants needed for temperature calculation
  * This only needs to be performed once as the constants are hard-coded for
  * each sensor.
  */

  // Parameters for ambient temperature calculations
  // See 7.3.1 - Calculation of absolute chip temperature Ta (sensor
  // temperature)
  resolution_comp = pow(2.0, (3 - get_resolution()));
  k_t1_scale = (int16_t)(eeprom_buffer[KT_SCALE] & 0xF0) >> 4;
  k_t2_scale = (int16_t)(eeprom_buffer[KT_SCALE] & 0x0F) + 10;

  v_th = (float)twos_16(eeprom_buffer[VTH_H], eeprom_buffer[VTH_L]);
  v_th = v_th / resolution_comp;

  k_t1 = (float)twos_16(eeprom_buffer[KT1_H], eeprom_buffer[KT1_L]);
  k_t1 /= (pow(2, k_t1_scale) * resolution_comp);

  k_t2 = (float)twos_16(eeprom_buffer[KT2_H], eeprom_buffer[KT2_L]);
  k_t2 /= (pow(2, k_t2_scale) * resolution_comp);

  // Parameters for object temperature calculations
  // See 7.3.3 - Calculation of To
  tgc = (float)twos_8(eeprom_buffer[CAL_TGC]) / 32.0;
  emissivity =
      unsigned_16(eeprom_buffer[CAL_EMIS_H], eeprom_buffer[CAL_EMIS_L]) /
      32768.0;
  a_common =
      twos_16(eeprom_buffer[CAL_ACOMMON_H], eeprom_buffer[CAL_ACOMMON_L]);
  a_i_scale = (int16_t)(eeprom_buffer[CAL_AI_SCALE] & 0xF0) >> 4;
  b_i_scale = (int16_t)eeprom_buffer[CAL_BI_SCALE] & 0x0F;

  alpha_cp =
      unsigned_16(eeprom_buffer[CAL_alphaCP_H], eeprom_buffer[CAL_alphaCP_L]) /
      (pow(2.0, eeprom_buffer[CAL_A0_SCALE]) * resolution_comp);
  a_cp = (float)twos_16(eeprom_buffer[CAL_ACP_H], eeprom_buffer[CAL_ACP_L]) /
         resolution_comp;
  b_cp = (float)twos_8(eeprom_buffer[CAL_BCP]) /
         (pow(2.0, (float)b_i_scale) * resolution_comp);
}

// Config Reads
uint8_t MLX90621::get_resolution() {
  /**
  * Get the resolution bits of the sensor.
  *
  * 00 = 15-bit,
  * 01 = 16-bit,
  * 10 = 17-bit,
  * 11 = 18-bit
  *
  * @return Resolution bits of the sensor
  */
  return (get_config() & 0x30) >> 4;
}

uint16_t MLX90621::get_config() {
  /**
  * Read the sensor's configuration register.
  * See section 8.2.2.1 of the MLX90621 datasheet for meaning of each bit
  */

  Wire.beginTransmission(RAM_ADDRESS);
  Wire.write(READ_RAM);
  Wire.write(CONFIG_ADDRESS);
  Wire.write(0x00); // Address step
  Wire.write(0x01); // Single read
  Wire.endTransmission(false);
  Wire.requestFrom(RAM_ADDRESS, 2);
  byte configLow = Wire.read();
  byte configHigh = Wire.read();
  uint16_t config = ((uint16_t)(configHigh << 8) | configLow);
  return config;
}

void MLX90621::check_configuration() {
  /**
  * Reloads the sensor configuration data if the sensor has not been
  * initialised.
  * A power reset or brown-out may cause the configuration data to be lost
  * during operation.
  * This check ensures that the sensor has been configured and is ready for
  * sensor reads.
  */
//  if (needs_reload()) {
    load_sensor();
//  }
}

bool MLX90621::needs_reload() {
  /**
  * Check the ready status of the sensor to see if it needs to be reloaded
  * The POR bit in the sensor's configuration is set if the device has been
  * initialised.
  * If the POR bit is clear, then the sensor has restarted and needs to be
  * reloaded
  *
  * @return Status of the sensor. {True: sensor reload required; False: sensor
  * has been initialised}
  */
  bool check = !(get_config() & 0x0400) >> POR_BIT;
  return check;
}

// Utilities
int16_t MLX90621::twos_16(uint8_t highByte, uint8_t lowByte) {
  /**
  * Return the 2's compliment of a 16-bit integer
  * @param highByte MSB of the integer
  * @param lowByte LSB of the integer
  * @return 2's compliment of the input integer
  */

  uint16_t combined_word = 256 * highByte + lowByte;
  if (combined_word > 32767)
    return (int16_t)(combined_word - 65536);
  return (int16_t)combined_word;
}

int8_t MLX90621::twos_8(uint8_t byte) {
  /**
  * Return the 2's compliment of an 8-bit integer
  * @param byte Byte to transform
  * @return 2's compliment of the input integer
  */
  if (byte > 127)
    return (int8_t)byte - 256;
  return (int8_t)byte;
}

uint16_t MLX90621::unsigned_16(uint8_t highByte, uint8_t lowByte) {
  /**
  * Combine two bytes into an unsigned word.
  * @param highByte Most significant byte
  * @param lowByte Least significant byte
  * @param Unsigned word of the combined bytes
  */
  return (highByte << 8) | lowByte;
}

///////////////////////////////////////////////////////////////////////////////
// Ambient Temperature

float MLX90621::get_ambient_temperature() {
  /**
  * Read the sensor and calculate the ambient temperature.
  * The calculated temperature is stored in a class variable to provide access
  * to other functions
  * @return Ambient temperature measured by the sensor in deg C.
  */

  check_configuration();
  float ptat = float(get_PTAT());

  // MLX90621 Datasheet - 7.3.1 - Calculation of absolute chip temperature Ta
  float ambient_temperature =
      ((-k_t1 + sqrt((k_t1 * k_t1) - (4 * k_t2 * (v_th - ptat)))) /
       (2 * k_t2)) +
      25.0;
  return ambient_temperature;
}

int MLX90621::get_PTAT() {
  /**
  * Read Proportional to Absolute Temperature sensor to find the ambient
  * temperature of the chip
  * @return Raw PTAT data from the sensor
  */
  Wire.beginTransmission(RAM_ADDRESS);
  Wire.write(READ_RAM);
  Wire.write(PTAT_ADDRESS);
  Wire.write(0x00); // Address step
  Wire.write(0x01); // Number of reads
  Wire.endTransmission(false);
  Wire.requestFrom(RAM_ADDRESS, 2);

  byte ptatLow = Wire.read();
  byte ptatHigh = Wire.read();
  return (ptatHigh * 256) + ptatLow;
}

///////////////////////////////////////////////////////////////////////////////
// Object temperature

void MLX90621::get_temperatures(float output_buffer[NUM_PIXELS]) {
  /**
  * Get the temperatures recorded by the sensor and insert them into the given
  * buffer
  * The buffer must be at least 64 items wide to fit the data.
  *
  * @param output_buffer array to insert the temperature into.
  */
  int ir_data[NUM_PIXELS];
  precalculate_frame_values();
  get_IR(ir_data);
  for (int i = 0; i < NUM_PIXELS; i++) {
    output_buffer[i] = calculate_pixel(i, ir_data);
  }
}

void MLX90621::get_temperatures(float output_buffer[NUM_ROWS][NUM_COLS],
                                bool mirror_frame) {
  /**
  * Get the temperatures recorded by the sensor and insert them into the given
  * buffer
  * A 2D buffer is needed to put the data in their proper pixel locations.
  *
  * @param output_buffer A 2D matrix the same size as the sensor frame to insert
  * the temperatures into
  * @param mirror_frame Option to flip the frame's output horizontally (for
  * looking through the lens)
  */
  int ir_data[NUM_PIXELS];
  precalculate_frame_values();
  get_IR(ir_data);

  for (int j = START_COL; j < START_COL + NUM_COLS; j++) {
    for (int i = START_ROW; i < START_ROW + NUM_ROWS; i++) {
      if (mirror_frame) {
        output_buffer[i][NUM_COLS - (j + 1)] =
            calculate_pixel((j * NUM_ROWS) + i, ir_data);
      } else {
        output_buffer[i - START_ROW][j - START_COL] = calculate_pixel((j * NUM_ROWS) + i, ir_data);
      }
    }
  }
}

void MLX90621::print_temperatures(HardwareSerial &ser) {
  /**
  * Print the temperature values of all pixels to the selected serial interface.
  * Temperatures are printed as a comma-separated list.
  * Pixels are ordered by row.
  *
  * @param ser The serial interface to print the temperatures to.
  */
  int ir_data[NUM_PIXELS];
  precalculate_frame_values();
  get_IR(ir_data);

  for (int i = START_PIXEL; i < START_PIXEL + NUM_PIXELS; i++) {
    ser.print(calculate_pixel(i, ir_data), 2);
    if (i < START_PIXEL + NUM_PIXELS - 1) {
      ser.print(',');
    }
  }
}

void MLX90621::precalculate_frame_values() {
  /**
  * Calulate the frame constants needed for pixel temperature compensation.
  * Must be run before measuring each frame.
  */

  ambient = get_ambient_temperature();
  int cpix = get_compensation_pixel();

  v_cp_off_comp = (float)cpix - (a_cp + b_cp * (ambient - 25.0));
  tak4 = pow((float)ambient + 273.15, 4.0);
}

int MLX90621::get_compensation_pixel() {
  /**
  * Get the value of the compensation pixel from the sensor.
  * Not entirely sure what this one does, but the datasheet does lots of maths
  * with it.
  *
  * @return Value of the compensation pixel
  */
  Wire.beginTransmission(RAM_ADDRESS);
  Wire.write(READ_RAM);
  Wire.write(CPIX_ADDRESS);
  Wire.write(0x00); // Address step
  Wire.write(0x01); // Number of reads
  Wire.endTransmission(false);
  Wire.requestFrom(RAM_ADDRESS, 2);
  byte cpixLow = Wire.read();
  byte cpixHigh = Wire.read();
  int cpix = twos_16(cpixHigh, cpixLow);
  return cpix;
}

void MLX90621::get_IR(int ir_buffer[]) {
  /**
  * Read in raw IR temperature data from the sensor's RAM
  * The Wire library cannot handle packets larger than 32-bytes, so the registry
  * must be read in chunks.
  *
  * @param ir_buffer Buffer to store the data. Must be at least NUM_PIXELS wide.
  */

  // Read in blocks of 32 bytes to overcome Wire buffer limit
  for (int j = START_PIXEL; j < START_PIXEL + NUM_PIXELS; j += (PACKET_SIZE / 2)) {
    Wire.beginTransmission(RAM_ADDRESS);
    Wire.write(READ_RAM);
    Wire.write(j);           // Starting address of 32-bit block
    Wire.write(0x01);        // Address step
    Wire.write(PACKET_SIZE); // 32 reads

    Wire.endTransmission(false);
    Wire.requestFrom(RAM_ADDRESS, PACKET_SIZE);

    for (int i = 0; i < (PACKET_SIZE / 2); i++) {
      uint8_t pixelDataLow = (uint8_t)Wire.read();
      uint8_t pixelDataHigh = (uint8_t)Wire.read();
      ir_buffer[j - START_PIXEL + i] = twos_16(pixelDataHigh, pixelDataLow);
    }
  }
}

float MLX90621::calculate_pixel(uint8_t pixel_num, int ir_data[]) {
  /**
  * Calculate the temperature recorded by the sensor for a given pixel.
  *
  * @param pixel_num Number of the pixel to calculate. [0-63]. First row sits
  * between 0-15; second row is between
  * 16-31, etc.
  * @param ir_data array containing the raw IR data from the sensor
  * @return The recorded temperature of the pixel in deg C.
  */

  // Grab the calibration and offset values for the individual pixel
  double a_ij =
      ((float)a_common + eeprom_buffer[pixel_num] * pow(2.0, a_i_scale)) /
      resolution_comp;
  double b_ij = (float)twos_8(eeprom_buffer[0x40 + pixel_num]) /
                (pow(2.0, b_i_scale) * resolution_comp);
  double v_ir_off_comp =
      (float)ir_data[pixel_num] - (a_ij + b_ij * (ambient - 25.0));
  double v_ir_tgc_comp = (float)v_ir_off_comp - tgc * v_cp_off_comp;

  double alpha_ij =
      ((float)unsigned_16(eeprom_buffer[CAL_A0_H], eeprom_buffer[CAL_A0_L]) /
       pow(2.0, (float)eeprom_buffer[CAL_A0_SCALE]));
  alpha_ij += ((float)eeprom_buffer[0x80 + pixel_num] /
               pow(2.0, (float)eeprom_buffer[CAL_DELTA_A_SCALE]));
  alpha_ij = alpha_ij / resolution_comp;

  // ksta = (float) twos_16(eeprom_buffer[CAL_KSTA_H],
  // eeprom_buffer[CAL_KSTA_L]) / pow(2.0, 20.0);
  // alpha_comp = (1 + ksta * (Tambient - 25.0)) * (alpha_ij - tgc * alpha_cp);
  double alpha_comp =
      (alpha_ij -
       tgc * alpha_cp); // For my MLX90621 the ksta calibrations were 0
                        // so I can ignore them and save a few cycles
  float v_ir_comp = v_ir_tgc_comp / emissivity;
  double temperature = pow((v_ir_comp / alpha_comp) + tak4, 1.0 / 4.0) - 273.15;

  return temperature;
}
