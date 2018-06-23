#ifndef RACE_DATA_LOGGER_H_
#define RACE_DATA_LOGGER_H_

#define DEBUG_PORT_TYPE NeoHWSerial
#define DEBUG_PORT NeoSerial

#define TCA0_ADDR 0x70
#define MPU_addr 0x53

#define RED_LED 12
#define GREEN_LED 13
#define TCA1_SELECT 51
#define sd_card_select 53

#define MPU_TCA_PORT 0

#define MLX90621_REFRESH_RATE 4
#define IR_ARRAY_COUNT 3
#define IR_ARRAY_TCA_START_PORT 1

//set TARGET refresh rates
#define sd_card_flush_rate_hz 1
#define sd_card_file_rollover_rate 43200000  //12 hours

#define NMEAGPS_FIX_MAX 2


//calculate timer values
#define onehzloop_refresh_millis 1000
#define fivehzloop_refresh_millis 200
#define tenhzloop_refresh_millis 100


//error codes
//0 = no errors
//1 = adxl345 mpu init
//2 = sd card init
//4 = could not open file
//8 = could not write data to file
//16 = gps not available
//32 = gps lockup
short last_error_code = 0;


unsigned long onehzloop_previous_millis;
unsigned long fivehzloop_previous_millis;
unsigned long tenhzloop_previous_millis;
unsigned long sd_card_last_rollover;
short sd_card_write_retries = 0;
unsigned long sample_count = 0;
char time_string[12];
char date_string[10];


//accelerometer setup
int16_t ax, ay, az;


//sd setup
SdFat SD;
File sd_card_log_file;
char filename[] = "log000.csv";


//gps setup
static NMEAGPS gps;
static gps_fix fix_data;
short previous_gps_seconds;
short previous_gps_cs;


//mlx90621 setup
MLX90621 ir_array[IR_ARRAY_COUNT];
unsigned short ir_array_loop_counter;

//functions setup
void printL(Print &outs, int32_t degE7);
void tca_select (uint8_t a);
void GPSisr(uint8_t c);
void create_log_file();
void reset_gps();
void check_for_gps_lockup();
void mlx90621_init();
void mlx90621_loop(int ir_array_id);
static void GPSloop();
static void accelerometer_loop();
static void ir_temp_loop(int ir_addr);
void write_to_sd_card();


#endif
