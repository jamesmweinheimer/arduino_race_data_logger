#include "rdl_container.h"

void rdl::clear_rdl_values() { 
	for(int a=0; a<12; a++) {
		time_string[a] = '0';
	}

	ax = NULL;
	ay = NULL;
	az = NULL;

	latitudeL = NULL;
	latitudeL = NULL;

	altitude = NULL;
	speed_mph = NULL;
	heading = NULL;

	for(int b=0; b<4; b++) {
		for(int c=0; c<32; c++) {
			tire_temp[b][c] = NULL;
		}
	}

	/*
	for(int d=0; d<4; d++) {
		tire_press[d] = NULL;
	}

	for(int e=0; e<4; e++) {
		brake_temp[e] = NULL;
	}

	for(int f=0; f<4; f++) {
		wheel_position[f] = NULL;
	}

	steering_angle = NULL;

	for(int g=0; g<2; g++) {
		brake_pressure[g] = NULL;
	}

	short ride_height = NULL;
	*/

	ir_temperature = NULL;

	uint8_t error_code = NULL;
}
























