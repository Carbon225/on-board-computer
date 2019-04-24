#ifndef _BMP180SENSOR_H_
#define _BMP180SENSOR_H_

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"

#include "Arduino.h"
// #include <Wire.h>
#include "BMP180.h"
#include "RemoteDebug.h"
#include "Sensor.h"
#include "DataQueue.h"

extern RemoteDebug Debug;
extern SemaphoreHandle_t i2c_mutex;

class BMP180Sensor : public Sensor, private Adafruit_BMP085 {
private:
	bool _started = false;
	double _referencePressure = 101111;

protected:
	virtual DataQueue::QueueElement read() {
		if (_started) {
			if (i2c_mutex != NULL) {
				
				int count = 0;
				while (!xSemaphoreTake(i2c_mutex, 0 / portTICK_PERIOD_MS)) {
					if (count > 10) {
						// debugE("BMP I2C blocked");
						// Sensor::sendToQueues(ErrorTypeToElement(ErrorTypes::I2CBlocked));
					}
					count++;
					vTaskDelay(10 / portTICK_PERIOD_MS);
				}

				// debugD("Reading BMP");
				
				float realTemperature = 11.11; // Adafruit_BMP085::readTemperature();
				int32_t realPressure = Adafruit_BMP085::readRawPressure(); // Adafruit_BMP085::readPressure();

				// debugD("BMP Temp = %g\nBMP Press = %d\n", realTemperature, realPressure);

				// give back semaphore
				xSemaphoreGive(i2c_mutex);

				DataQueue::DataUnion data;
				data.floatValue = realTemperature;

				DataQueue::QueueElement element = {
					.type = DataTypes::TemperatureBMP,
					.data = data,
					.time = (uint16_t) (millis() / 1000)
				};

				// queue temperature element
				Sensor::sendToQueues(element);

				element.type = DataTypes::PressureBMP;
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
    BMP180Sensor(const char *const pcName)
    : Sensor(pcName), Adafruit_BMP085() {
		
    }

	void start() {
        debugD("Starting BMP");

		if (i2c_mutex != NULL) {
			// wait for i2c semaphore
			while (!xSemaphoreTake(i2c_mutex, 10 / portTICK_PERIOD_MS)) {
				// debugE("MS start I2C blocked");
				// Sensor::sendToQueues(ErrorTypeToElement(ErrorTypes::I2CBlocked));
			}
			// start sensor and give back semaphore
			while (!Adafruit_BMP085::begin()) {
				debugE("BMP start failed");
				// xSemaphoreGive(i2c_mutex);
				// return;
				vTaskDelay(5 / portTICK_PERIOD_MS);
			}

			xSemaphoreGive(i2c_mutex);
				
			_started = true;
			debugI("BMP started");
		} else {
			debugE("BMP start semaphore null");
			// Sensor::sendToQueues(ErrorTypeToElement(ErrorTypes::SemaphoreNULL));
		}
    }

	void stop() {
        Sensor::stop();
    }
};



#endif
