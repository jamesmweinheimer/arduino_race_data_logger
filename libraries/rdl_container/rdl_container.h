#ifndef RDL_CONTAINER_H_
#define RDL_CONTAINER_H_

#include <NMEAGPS.h>

struct rdl {
	char time_string[12];

	float ax;
	float ay;
	float az;

	//get type from NMEAGPS.h
	float latitudeL;
	float longitudeL;
	uint8_t altitude;
	unsigned short speed_mph; //def as float in GPSfix.h
	unsigned short heading; //def as float in GPSfix.h

	float tire_temp[4][32];



	
	//unsigned short tire_press[4];
	
	//unsigned short brake_temp[4];

	//short wheel_position[4];

	//short steering_angle;

	//unsigned short brake_pressure[2];

	//unsigned short ride_height;

	unsigned short ir_temperature;

	uint8_t error_code;

	void clear_rdl_values();
	
};


#endif
