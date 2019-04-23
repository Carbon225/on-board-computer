#ifndef _TMP102SENSOR_H_
#define _TMP102SENSOR_H_

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"

#include "Arduino.h"
#include "SparkFunTMP102.h"
#include "RemoteDebug.h"

#include "Sensor.h"
#include "DataQueue.h"

extern RemoteDebug Debug;
extern SemaphoreHandle_t i2c_mutex;

class TMP102Sensor : public Sensor, private TMP102 {
private:
	bool _started = false;

protected:
	virtual DataQueue::QueueElement read() {
		if (_started) {
			if (i2c_mutex != NULL) {
				//  wait for i2c semaphore
				int count = 0;
				while (!xSemaphoreTake(i2c_mutex, 0 / portTICK_PERIOD_MS)) {
					if (count > 10) {
						// debugE("TMP I2C blocked");
						// Sensor::sendToQueues(ErrorTypeToElement(ErrorTypes::I2CBlocked));
					}
					count++;
					vTaskDelay(10 / portTICK_PERIOD_MS);
				}

				// read sensor and give back semaphore
				TMP102::wakeup();
				float temp = TMP102::readTempC();
				TMP102::sleep();

				xSemaphoreGive(i2c_mutex);

				DataQueue::DataUnion data;
				data.floatValue = temp;

				DataQueue::QueueElement element = {
					.type = DataTypes::TemperatureTMP,
					.data = data,
					.time = (uint16_t) (millis() / 1000)
				};

				// save temperature element
				return element;
				/*
					} else {
						debugE("TMP I2C blocked");
						return ErrorTypeToElement(ErrorTypes::I2CBlocked);
					}
				*/
			} else {
				debugE("TMP semaphore null");
				return ErrorTypeToElement(ErrorTypes::SemaphoreNULL);
			}
		} else {
			debugE("TMP not started");
			return ErrorTypeToElement(ErrorTypes::TMPNotStarted);
		}
	}
public:
	TMP102Sensor(const char *const pcName)
	: Sensor(pcName), TMP102(0x48) {
		
	}

	void start() {
        debugD("Starting TMP...\n");

		if (i2c_mutex != NULL) {
			// wait for i2c semaphore to start sensor
			while (!xSemaphoreTake(i2c_mutex, 100 / portTICK_PERIOD_MS)) {
				// debugE("TMP start I2C blocked");
				// Sensor::sendToQueues(ErrorTypeToElement(ErrorTypes::I2CBlocked));
			}
		
			TMP102::begin();
			TMP102::wakeup();

			xSemaphoreGive(i2c_mutex);

			_started = true;
			debugI("TMP started");
		} else {
			debugE("TMP start semaphore null");
			Sensor::sendToQueues(ErrorTypeToElement(ErrorTypes::SemaphoreNULL));
		}
    }

	void stop() {
        Sensor::stop();
    }
};


#endif
