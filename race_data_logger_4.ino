#include <Wire.h>
#include <NMEAGPS.h>
#include <NeoHWSerial.h>
#include "GPSport.h"
#include <SdFat.h>
#include <MLX90621_2.h>
#include <rdl_container.h>
#include "race_data_logger.h"

rdl data_container;


//=======================================================================================
// Change i2c multiplexer device
//=======================================================================================
void tca_select (uint8_t a) {
  if (a < 0 || a > 7) return;

  Wire.beginTransmission(TCA0_ADDR);
  Wire.write(1 << a);
  Wire.endTransmission();
}


//=======================================================================================
// Stream bits from serial interface from GPS device
//=======================================================================================
void GPSisr(uint8_t c) {
  gps.handle(c);
}


//=======================================================================================

void create_log_file() {
  //open log file on SD card
  //increment filename
  for (short i = 0; i < 1000; i++) {
    sprintf(filename, "log%03d.csv", i);
    if (!SD.exists(filename)) {
      // only open a new file if it doesn't exist
      sd_card_log_file = SD.open(filename, FILE_WRITE);

      DEBUG_PORT.print(F("Logging to "));
      DEBUG_PORT.println(filename);

      /*
      DEBUG_PORT.print(F("Writing file timestamp..."));

      //set file modification date time
      if (!sd_card_log_file.timestamp(T_WRITE, fix_data.dateTime.year, fix_data.dateTime.month, fix_data.dateTime.date, fix_data.dateTime.hours, fix_data.dateTime.minutes, fix_data.dateTime.seconds)) {
        DEBUG_PORT.print(F("failed\n"));
      }
      else {
        DEBUG_PORT.print(F("OK\n"));
      }
      */
      break; // leave the loop
    }
  }

  //if the file opened successfully
  if(!sd_card_log_file) {
    // if the file didn't open, print an error:
    DEBUG_PORT.println(F("Error writing to file!"));

    data_container.error_code |= 2;
  }

}


//=======================================================================================
void reset_gps() {
    DEBUG_PORT.print(F("resetting..."));
    
    gps.reset();

    //resend initialization string
    gps.send_P(&gps_port, F("PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0")); 

    data_container.error_code -= 32;    
}

//=======================================================================================
void check_for_gps_lockup() {
  if(fix_data.dateTime.seconds == previous_gps_seconds && fix_data.dateTime_cs == previous_gps_cs) {
    DEBUG_PORT.print(F("GPS lockup..."));

    data_container.error_code |= 32;

    //set flag for reporting error status with next log file line written
    reset_gps();
  }

  previous_gps_seconds = fix_data.dateTime.seconds;
  previous_gps_cs = fix_data.dateTime_cs;
}


//=======================================================================================
static void GPSloop() {
  sprintf(data_container.time_string, "%02i:%02i:%02i.%03i", fix_data.dateTime.hours, fix_data.dateTime.minutes, fix_data.dateTime.seconds, fix_data.dateTime_cs);

  data_container.latitudeL = fix_data.latitudeL()/10000000.0;
  data_container.longitudeL = fix_data.longitudeL()/10000000.0;
  data_container.speed_mph = fix_data.speed_mph();
  data_container.heading = fix_data.heading();
}


//=======================================================================================
static void accelerometer_loop() {
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x32);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 6);
  data_container.ax = (Wire.read()|Wire.read()<<8)/256.0;
  data_container.ay = (Wire.read()|Wire.read()<<8)/256.0;
  data_container.az = (Wire.read()|Wire.read()<<8)/256.0;
  
//  printf(data_container.ax, "%3i", (Wire.read()|Wire.read()<<8)/2048.0;
//  printf(data_container.ay, "%3i", (Wire.read()|Wire.read()<<8)/2048.0;
//  printf(data_container.az, "%3i", (Wire.read()|Wire.read()<<8)/2048.0;

/*
    // display tab-separated accel/gyro x/y/z values
    DEBUG_PORT.print(ax/2048.0,3); DEBUG_PORT.print("\t");
    DEBUG_PORT.print(ay/2048.0,3); DEBUG_PORT.print("\t");
    DEBUG_PORT.print(az/2048.0,3); DEBUG_PORT.print("\t");

      sd_card_log_file.print(ax/2048.0,3);          sd_card_log_file.print(","); 
      sd_card_log_file.print(ay/2048.0,3);          sd_card_log_file.print(","); 
      sd_card_log_file.print(az/2048.0,3);          sd_card_log_file.print(","); 
*/  
}


//=======================================================================================
static void ir_temp_loop (int ir_addr) {
  Wire.beginTransmission(TCA0_ADDR);
  Wire.write(1 << ir_addr);
  Wire.endTransmission();  
  
  Wire.beginTransmission(0x5A);
  Wire.write(0x07);
  Wire.endTransmission(false);
  Wire.requestFrom(0x5A, 3);
  double temperature = Wire.read() | Wire.read() << 8;
  
  float temperature1 = (temperature * 0.02 - 273.15) * 9 / 5 + 32;

  data_container.ir_temperature = temperature1;
}


//=======================================================================================
void mlx90621_init() {
  DEBUG_PORT.println("Initializing mlx90621...");

  for(int i=0; i<IR_ARRAY_COUNT; i++) {
    DEBUG_PORT.print(i);
    DEBUG_PORT.print(" ");    

    tca_select(i+IR_ARRAY_TCA_START_PORT);    
    
    /*
    if(ir_array[i].initialise(MLX90621_REFRESH_RATE)) {
        DEBUG_PORT.println(" OK");
    }
    else {
        DEBUG_PORT.println(" failed");
    }
    */
   
    Wire.begin();
    ir_array[i].initialise(MLX90621_REFRESH_RATE);
  }

  DEBUG_PORT.println(" ");
}

//=======================================================================================
void mlx90621_loop(int ir_array_id) {

    tca_select(ir_array_id + IR_ARRAY_TCA_START_PORT);

    ir_array[ir_array_id].get_temperatures(data_container.tire_temp[ir_array_id]);

}


//=======================================================================================
void write_to_sd_card() {
  sd_card_log_file.print(data_container.time_string);     sd_card_log_file.print(",");
  sd_card_log_file.print(data_container.ax,2);            sd_card_log_file.print(",");
  sd_card_log_file.print(data_container.ay,2);            sd_card_log_file.print(",");
  sd_card_log_file.print(data_container.az,2);            sd_card_log_file.print(",");
  sd_card_log_file.print(data_container.latitudeL,10);    sd_card_log_file.print(",");
  sd_card_log_file.print(data_container.longitudeL,10);   sd_card_log_file.print(",");
  sd_card_log_file.print(data_container.altitude);        sd_card_log_file.print(",");
  sd_card_log_file.print(data_container.speed_mph,1);     sd_card_log_file.print(",");
  sd_card_log_file.print(data_container.heading);         sd_card_log_file.print(",");
  for(int j=0; j<IR_ARRAY_COUNT; j++) {
    for(int i=0; i<16; i++) {
        sd_card_log_file.print((data_container.tire_temp[j][i] + data_container.tire_temp[j][i+16])/2.0);   sd_card_log_file.print(",");
    }
  }
  
  sd_card_log_file.print(data_container.error_code);    
  sd_card_log_file.print("\n");

  data_container.clear_rdl_values();
  
}


//=======================================================================================
void write_to_screen() {
  DEBUG_PORT.print(data_container.time_string);   DEBUG_PORT.print("\t");
  DEBUG_PORT.print(data_container.ax,2);            DEBUG_PORT.print("\t");
  DEBUG_PORT.print(data_container.ay,2);            DEBUG_PORT.print("\t");
  DEBUG_PORT.print(data_container.az,2);            DEBUG_PORT.print("\t");
  DEBUG_PORT.print(data_container.latitudeL,10);     DEBUG_PORT.print("\t");
  DEBUG_PORT.print(data_container.longitudeL,10);    DEBUG_PORT.print("\t");
  DEBUG_PORT.print(data_container.altitude);      DEBUG_PORT.print(",");
  DEBUG_PORT.print(data_container.speed_mph,1);     DEBUG_PORT.print(",");
  DEBUG_PORT.print(data_container.heading);       DEBUG_PORT.print(",");
  for(int j=0; j<IR_ARRAY_COUNT; j++) {
    for(int i=0; i<16; i++) {
        DEBUG_PORT.print((data_container.tire_temp[j][i] + data_container.tire_temp[j][i+16])/2.0);   DEBUG_PORT.print(",");
    }
  }
  DEBUG_PORT.print(data_container.error_code);    
  DEBUG_PORT.print("\n");

  //data_container.clear_rdl_values();
}

//=======================================================================================
// 
//=======================================================================================
void setup() {
  // Start the normal trace output
  DEBUG_PORT.begin(115200);

  pinMode(GREEN_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);  
  digitalWrite(GREEN_LED, HIGH);
  digitalWrite(RED_LED, HIGH);


  //=======================================================
  // mlx90621 setup
  //=======================================================
  mlx90621_init();
  ir_array_loop_counter = 0;

  //=======================================================
  // gps setup
  //=======================================================
  // Start the GPS device
  gps_port.attachInterrupt(GPSisr);
  gps_port.begin(9600);
    
  //set gps to higher serial baud rate
  gps_port.println(F("$PMTK251,115200*1F\r\n"));
  delay(1000);
  gps_port.begin(115200);

  //set gps to generate data at 10Hz
  gps_port.println(F("$PMTK220,100*2F"));
  delay(1000);  

  //configure the GPS.  These are commands for MTK GPS devices.  
  //other brands have different commands.
  // RMC only
  gps.send_P(&gps_port, F("PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0")); 

  DEBUG_PORT.print(F("Initializing GPS..."));

  int gps_init_retries = 0;
  
  while(!gps.available()) {
    delay(100);
    DEBUG_PORT.print('.');

    gps_init_retries++;

    if(gps_init_retries == 10) {
      break;
    }
  }

  if(gps.available()) {
    fix_data = gps.read();
  
    DEBUG_PORT.print(F("OK\n"));
  
    DEBUG_PORT.print(F("Current UTC date/time "));
    sprintf(time_string, "%02i/%02i/%04i %02i:%02i:%02i", fix_data.dateTime.date, fix_data.dateTime.month, fix_data.dateTime.year, fix_data.dateTime.hours, fix_data.dateTime.minutes, fix_data.dateTime.seconds);
    DEBUG_PORT.print(time_string);
    DEBUG_PORT.print('\n');
  }
  else {
    DEBUG_PORT.print(F("failed\n"));
    data_container.error_code |= 16;
  }

  //initialize variables to use for checking for gps lockup
  previous_gps_seconds = fix_data.dateTime.seconds;
  previous_gps_cs = fix_data.dateTime_cs;

  //=======================================================
  // adxl345 setup
  //=======================================================
  //join I2C bus
  Wire.begin();

  //
  tca_select(MPU_TCA_PORT);
  
  // initialize device
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x2D);
  Wire.write(8);  
  Wire.endTransmission(false);  

  // verify connection
  DEBUG_PORT.print(F("Initializing ADXL345..."));
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x00);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 1);
   
  if(Wire.read() == 0xE5) {
    DEBUG_PORT.print("OK\n");
  }
  else {
    DEBUG_PORT.print(F("failed!\n"));
    data_container.error_code = 1;
  }


  //set accelerometer to full scale
  
  //set calibration offset values

  //set accelerometer to Gs as unit of measurement
 

  //=======================================================
  // sd card setup
  //=======================================================
  //how many times did we have to retry to get the sd card initialized?
  short sd_init_retries = 0;

  //initialize SD card by setting pin 53 high
  DEBUG_PORT.print(F("Initializing SD card..."));
  pinMode(sd_card_select, OUTPUT);
  digitalWrite(sd_card_select, HIGH);

  //initialize the SD card
  while (!SD.begin(sd_card_select)) {
    //if SD card isn't initialized, send user message then retry
    DEBUG_PORT.print(".");

    sd_init_retries++;
    
    //allow 5 retries then stop retrying
    if(sd_init_retries > 4) {
      DEBUG_PORT.print(F("failed!\n"));
      data_container.error_code |= 2;

      return;
    }
  }

  //SD card initialized properly
  //send message to user
  DEBUG_PORT.print(F("OK\n"));

  create_log_file();
  
  // GPS Visualizer requires a header to identify the CSV fields.
  sd_card_log_file.print("Date: ");
  sd_card_log_file.print(fix_data.dateTime.date);
  sd_card_log_file.print(fix_data.dateTime.month);
  sd_card_log_file.print(fix_data.dateTime.year);
  sd_card_log_file.print("\n");
  sd_card_log_file.print(F("time,ax,ay,az,latitude,longitude,altitude,speed,heading,tire_temp1,,,,,,,,,,,,,,,,tire_temp2,,,,,,,,,,,,,,,,tire_temp3,,,,,,,,,,,,,,,,tire_temp4,,,,,,,,,,,,,,,,error_code\n")); 
  

  //=======================================================
  // misc setup
  //=======================================================  
  tenhzloop_previous_millis = fivehzloop_previous_millis = onehzloop_previous_millis = sd_card_last_rollover = millis();

  digitalWrite(GREEN_LED, LOW);
  digitalWrite(RED_LED, LOW);

  DEBUG_PORT.print(F("Logging...\n\n"));
}


//=======================================================================================
void loop() {
 
  if(gps.overrun()) {
    gps.overrun(false);
  }

  if(data_container.error_code > 0) {
    if(!last_error_code == data_container.error_code) {
      DEBUG_PORT.print(F("Error code "));
      DEBUG_PORT.println(data_container.error_code);
    }
 
    last_error_code = data_container.error_code;

    //turn on red light
    digitalWrite(RED_LED, HIGH);
  }
  else {
    last_error_code = 0;
    
    //turn off red light
    digitalWrite(RED_LED, LOW);    
  }

  if(gps.available()) {
    fix_data = gps.read();
  }

  //=======================================================
  // ten hz events
  //=======================================================    
  if (millis()-tenhzloop_previous_millis >= tenhzloop_refresh_millis) {

    
    //DEBUG_PORT.println((millis()-tenhzloop_previous_millis));

    
    tenhzloop_previous_millis = millis();

    //the important stuff
    GPSloop();
  
    tca_select(MPU_TCA_PORT);
    accelerometer_loop();

    /*
    for(int i=0; i<2; i++) {
      ir_temp_loop(i);
    }
    */
    
    //write out contents of data_container
    write_to_screen();
    write_to_sd_card();

  }

  
  //=======================================================
  // five hz events
  //=======================================================    
  if (millis()-fivehzloop_previous_millis >= fivehzloop_refresh_millis) {
    fivehzloop_previous_millis = millis();
    
    //blink the green light as an activity light
    digitalWrite(GREEN_LED, HIGH);

    //get tire temperature data
    mlx90621_loop(ir_array_loop_counter);

    ir_array_loop_counter++;

    if(ir_array_loop_counter == IR_ARRAY_COUNT) {
      ir_array_loop_counter = 0;
    }

  }  


  //=======================================================
  // one hz events
  //=======================================================    
  if(millis()-onehzloop_previous_millis >= onehzloop_refresh_millis) {
    onehzloop_previous_millis = millis();
    sd_card_log_file.flush();

    check_for_gps_lockup();
  }


  //=======================================================
  // sd card log rollover after 12 hours
  //=======================================================    
  if(millis()-sd_card_last_rollover >= sd_card_file_rollover_rate) {
    sd_card_last_rollover = millis();
    sd_card_log_file.close();
    DEBUG_PORT.print(F("Log rolling over\n"));
    create_log_file();
  }


  digitalWrite(GREEN_LED, LOW);
}
