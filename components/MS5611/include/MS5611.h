/*
ms5611.h
Library for barometric pressure sensor MS5611-01BA on I2C with arduino

by Petr Gronat@2014
*/

// Include guard token - prevents to include header file twice
#ifndef MS5611_h
#define MS5611_h 	//create token

// Include Arduino libraries
#include "Arduino.h"
#include <Wire.h>

#define N_PROM_PARAMS 6


// address of the device MS5611
#define ADD_MS5611 0x76 	// can be 0x76 if CSB pin is connected to GND

class MS5611{
	public:
		MS5611();		//constructor
			void 		begin();
			uint32_t 	getRawTemperature();
			int32_t 	getTemperature();
			uint32_t 	getRawPressure();
			int32_t 	getPressure();
			void 		readCalibration();
			void 		getCalibration(uint16_t *);
			void 		sendCommand(uint8_t);
			uint32_t 	readnBytes(uint8_t);
	private:
			void 		reset();
		//variables
		int32_t 	m_P;
		int32_t  	m_T;
		int32_t 	m_dT;
		uint16_t 	m_C[N_PROM_PARAMS];
		uint32_t 	m_lastTime;
};

void carbonWait(uint32_t ms);

#endif
