#ifndef _GPSSENSOR_H_
#define _GPSSENSOR_H_


#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "sys/time.h"

#include "Sensor.h"
#include "DataQueue.h"
#include "TinyGPS++.h"
#include "RemoteDebug.h"


extern RemoteDebug Debug;


class GPSSensor : private TinyGPSPlus, public Sensor {
private:
	TaskHandle_t _loopTaskHandle = NULL;
	LocationData _location;

	static void loopTask(void *gpsSensorVoid) {
		// get class instance pointer
		GPSSensor *gpsSensor = (GPSSensor*) gpsSensorVoid;

		while (true) {
			unsigned char c = '\0';
			// try to read one byte
			while (uart_read_bytes(UART_NUM_1, &c, 1, 0) > 0) {
				// debugD("Got byte from GPS");
				// get gps data from read bytes
				if (gpsSensor->encode(c)) {
					// debugD("Got complete data from GPS");
					if (gpsSensor->location.isValid()) {
						// set current location
						gpsSensor->_location.lat = gpsSensor->location.lat();
						gpsSensor->_location.lng = gpsSensor->location.lng();
					} else {
						debugE("Location invalid");
					}

					if (gpsSensor->altitude.isValid()) {
						gpsSensor->_location.alt = (unsigned int) gpsSensor->altitude.meters();
					} else {
						debugE("Altitude invalid");
					}

					if (gpsSensor->time.isValid()) {
						debugV("Da time is %u:%u", gpsSensor->time.hour(), gpsSensor->time.minute());

						// update local RTC
						struct timeval tv;
						tv.tv_sec = gpsSensor->time.value();
						settimeofday(&tv, NULL);
						debugV("Set system time to %ld", ::time(NULL));
					} else {
						debugE("Location invalid");
					}
				}
			}

			if (millis() > 5000 && gpsSensor->charsProcessed() < 10)
			{
				debugE("No GPS found");
				vTaskDelay(500 / portTICK_PERIOD_MS);
			}
			vTaskDelay(100 / portTICK_PERIOD_MS);
		}
		gpsSensor->_loopTaskHandle = NULL;
		vTaskDelete(NULL);
	}

protected:
    /* you need to write this virutal function
       so the Sensor base class can use your sensor */
    virtual DataQueue::QueueElement read() {
        // create union for storing temperature
        DataQueue::DataUnion data;
        data.locationData = _location; // assign to correct type

        // create queue element with data and type
        DataQueue::QueueElement element = {
            .type = DataTypes::LocationData,  		// set type for this reading
            .data = data,							// our value union
			.time = (uint16_t) (millis() / 1000)
        };

        // return created element to add to queues
        return element;
    }

public:
    GPSSensor(const char *const pcName, int n_queues = 3)
	: Sensor(pcName, n_queues) {
    	_location = LocationData {
    		0, 0, 0
    	};
    	setup();
    }

    virtual ~GPSSensor() {
		// close serial port and kill loop task
    	uart_driver_delete(UART_NUM_1);
    	if (_loopTaskHandle != NULL)
    		vTaskDelete(_loopTaskHandle);
    }

    void setup() {
		debugD("GPS starting...");

		// serial port configuration
		uart_config_t uart_config = {
			.baud_rate = 9600,
			.data_bits = UART_DATA_8_BITS,
			.parity    = UART_PARITY_DISABLE,
			.stop_bits = UART_STOP_BITS_1,
			.flow_ctrl = UART_HW_FLOWCTRL_DISABLE
		};
		// open serial port
		uart_param_config(UART_NUM_1, &uart_config);
		uart_set_pin(UART_NUM_1, 9, 10, -1, -1);
		uart_driver_install(UART_NUM_1, 8*1024, 0, 0, NULL, 0);

		// start reading loop task
		xTaskCreate(loopTask, "gpsLoopTask", 8*1024, (void*) this, 5, &_loopTaskHandle);

		debugI("GPS started");
    }

	LocationData getLocation() {
		return _location;
	}
};


#endif /* _GPSSENSOR_H_ */
