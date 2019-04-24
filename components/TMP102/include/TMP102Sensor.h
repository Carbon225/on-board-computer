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
				vTaskDelay(10 / portTICK_PERIOD_MS);
				float temp = TMP102::readTempC();
				vTaskDelay(10 / portTICK_PERIOD_MS);
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
			vTaskDelay(10 / portTICK_PERIOD_MS);
			
			// Initialize sensor0 settings
			// These settings are saved in the sensor, even if it loses power

			// set the number of consecutive faults before triggering alarm.
			// 0-3: 0:1 fault, 1:2 faults, 2:4 faults, 3:6 faults.
			TMP102::setFault(3);  // Trigger alarm immediately

			// set the polarity of the Alarm. (0:Active LOW, 1:Active HIGH).
			TMP102::setAlertPolarity(1); // Active HIGH

			// set the sensor in Comparator Mode (0) or Interrupt Mode (1).
			TMP102::setAlertMode(0); // Comparator Mode.

			// set the Conversion Rate (how quickly the sensor gets a new reading)
			//0-3: 0:0.25Hz, 1:1Hz, 2:4Hz, 3:8Hz
			TMP102::setConversionRate(3);

			//set Extended Mode.
			//0:12-bit Temperature(-55C to +128C) 1:13-bit Temperature(-55C to +150C)
			TMP102::setExtendedMode(0);

			//set T_HIGH, the upper limit to trigger the alert on
			// sensor0.setHighTempF(85.0);  // set T_HIGH in F
			//sensor0.setHighTempC(29.4); // set T_HIGH in C

			//set T_LOW, the lower limit to shut turn off the alert
			// sensor0.setLowTempF(84.0);  // set T_LOW in F
			//sensor0.setLowTempC(26.67); // set T_LOW in C

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
