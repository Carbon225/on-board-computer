#ifndef _CANSATSPECIFICCODE_H_
#define _CANSATSPECIFICCODE_H_

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "RadioHAL.h"
#include "DataQueue.h"
#include "ESP32_Servo.h"
#include "RemoteDebug.h"

// created in main
extern RemoteDebug Debug;
extern DHT22Sensor *dht22; // (GPIO_NUM_4, "DHTn1");
extern PMS5003Sensor *pms5003;
extern MPUHAL *mpu6050;
extern MS5611Sensor *ms5611;
extern TMP102Sensor *tmp102;
extern GPSSensor *gps;

namespace Cansat {

	// interrupt
	void onReceive(int packetSize) {
		char cmd[16] = {'\0'};
		// receive from ground station
		LoRa.readBytes(cmd, packetSize);

		debugI("Received %s", cmd);

		if (strcmp(cmd, "sleep") == 0) {
			debugW("Shutting down sensors");

			// shutdown all sensors except the GPS
			delete dht22;
			delete pms5003;
			delete mpu6050;
			delete ms5611;
			delete tmp102;

			debugW("Going to sleep");

			esp_sleep_enable_timer_wakeup(6e7); // 1 minute in us
			esp_deep_sleep_start();

		} else if (strcmp(cmd, "reset") == 0) {
			debugW("Restarting...");
			ESP.restart();
		} else {
			debugE("Unknown command received");
		}
	}

	Servo valveServo;

	void setValveEnable(bool active) {
		if (active) {
			valveServo.attach(25);
		} else {
			valveServo.detach();
		}
	}

	void openValve() {
		debugW("Opening valve");
		valveServo.write(0);
	}

	void closeValve() {
		debugW("Closing valve");
		valveServo.write(90);
	}

	void valveTask(void*) {
		unsigned int highest_alt = 0;

		TickType_t xLastWakeTime;
		while (true) {
			xLastWakeTime = xTaskGetTickCount();

			// unsigned int alt = ms5611->getAltitide();
			unsigned int alt = gps->getLocation().alt;

			// filter bad readings
			if (alt > 50 && alt < 6000) {
				// if we are 300 meters below max altitude open valve
				if (alt > highest_alt) {
					highest_alt = alt;
				} else if (highest_alt - alt > 300) {
					setValveEnable(true);
					openValve();
					vTaskDelay(10000 / portTICK_PERIOD_MS);
					closeValve();
					setValveEnable(false);

					// after opening and closing valve kill task
					vTaskDelete(NULL);
				}
			}

			// vTaskDelayUntil(&xLastWakeTime, 300 / portTICK_PERIOD_MS);
			vTaskDelay(300 / portTICK_PERIOD_MS);
		}
		vTaskDelete(NULL);
	}

	void testServoTask(void*) {
		while (true) {
			vTaskDelay(1000 / portTICK_PERIOD_MS);
			setValveEnable(true);
			// debugW("Opening valve");
			openValve();

			vTaskDelay(1000 / portTICK_PERIOD_MS);
			// debugW("Closing valve");
			closeValve();
			vTaskDelay(500 / portTICK_PERIOD_MS);
			setValveEnable(false);
		}
		vTaskDelete(NULL);
	}

} // namespace Cansat


#endif /* _CANSATSPECIFICCODE_H_ */
