#ifndef _MS5611SENSOR_H_
#define _MS5611SENSOR_H_

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"

#include "Arduino.h"
// #include <Wire.h>
#include "MS5611.h"
#include "RemoteDebug.h"
#include "Sensor.h"
#include "DataQueue.h"

extern RemoteDebug Debug;
extern SemaphoreHandle_t i2c_mutex;

class MS5611Sensor : public Sensor, private MS5611 {
private:
	bool _started = false;
	double _referencePressure = 101111;
	double _lastPressure = _referencePressure;

protected:
	virtual DataQueue::QueueElement read() {
		if (_started) {
			// debugD("Getting semaphore");
			if (i2c_mutex != NULL) {
				
				int count = 0;
				while (!xSemaphoreTake(i2c_mutex, 0 / portTICK_PERIOD_MS)) {
					if (count > 10) {
						// debugE("MS I2C blocked");
						// Sensor::sendToQueues(ErrorTypeToElement(ErrorTypes::I2CBlocked));
					}
					count++;
					vTaskDelay(10 / portTICK_PERIOD_MS);
				}
				
				// try to get i2c semaphore
				// if  (xSemaphoreTake(i2c_mutex, 50 / portTICK_PERIOD_MS)) {
				// debugD("Reading MS");
				double realTemperature = 11.11; // (double)MS5611::getTemperature() / 100.0f;
				uint32_t realPressure = MS5611::getRawPressure(); // MS5611::getPressure();

				// give back semaphore
				xSemaphoreGive(i2c_mutex);
				// debugD("Done reading");

				// verify data
				/*if (realPressure < 40000 || realPressure > 120000) {
					realPressure = _lastPressure;
					// return ErrorTypeToElement(ErrorTypes::BadReading);
				}*/

				_lastPressure = realPressure;

				// debugV("Temperature = %d", realTemperature);
				// debugE("MS raw data = %u", realPressure);

				DataQueue::DataUnion data;
				data.doubleValue = realTemperature;

				DataQueue::QueueElement element = {
					.type = DataTypes::TemperatureMS,
					.data = data,
					.time = (uint16_t) (millis() / 1000)
				};

				// queue temperature element
				Sensor::sendToQueues(element);

				element.type = DataTypes::Pressure;
				element.data.longValue = realPressure;

				// queue pressure element
				return element;

				/*
					} else {
						// debugE("MS I2C blocked");
						return ErrorTypeToElement(ErrorTypes::I2CBlocked);
					}
				*/				
			} else {
				// debugE("MS semaphore null");
				return ErrorTypeToElement(ErrorTypes::SemaphoreNULL);
			}
		} else {
			// debugE("MS not started");
			return ErrorTypeToElement(ErrorTypes::MSNotStarted);
		}
	}

public:
    MS5611Sensor(const char *const pcName)
    : Sensor(pcName), MS5611() {
		
    }

	void start() {
        debugD("Starting MS");

		if (i2c_mutex != NULL) {
			// wait for i2c semaphore
			while (!xSemaphoreTake(i2c_mutex, 10 / portTICK_PERIOD_MS)) {
				// debugE("MS start I2C blocked");
				// Sensor::sendToQueues(ErrorTypeToElement(ErrorTypes::I2CBlocked));
			}
			// start sensor and give back semaphore
			MS5611::begin();

			xSemaphoreGive(i2c_mutex);
				
			_started = true;
			debugI("MS started");
		} else {
			debugE("MS start semaphore null");
			Sensor::sendToQueues(ErrorTypeToElement(ErrorTypes::SemaphoreNULL));
		}
    }

    // in meters
    unsigned int getAltitide() {
    	int32_t pressure = MS5611::getPressure();
    	// double altitude = MS5611::getAltitude(pressure);

    	return (unsigned int) 0;
    }

	void stop() {
        Sensor::stop();
    }
};


#endif
