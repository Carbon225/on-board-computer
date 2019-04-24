#ifndef _DATAQUEUE_H_
#define _DATAQUEUE_H_

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "sys/time.h"

#include "Arduino.h"
#include "TinyGPS++.h"
#include "RemoteDebug.h"

extern RemoteDebug Debug;

struct PMSData {
	uint16_t data1;
	uint16_t data2;
	uint16_t data3;
	uint16_t data4;

	uint16_t data5;
	uint16_t data6;
	uint16_t data7;
	uint16_t data8;

	uint16_t data9;
	uint16_t data10;
	uint16_t data11;
	uint16_t data12;

	uint16_t data13;
};

struct MPUData {
	float roll;
	float pitch;
	float yaw;

	float ax;
	float ay;
	float az;
};

struct LocationData {
	double lat;
	double lng;
	uint32_t alt;
};

namespace ErrorTypes {
	enum ErrorType {
		I2CBlocked,
		SemaphoreNULL,
		LoraBlocked,
		MSNotStarted,
		TMPNotStarted,
		BadReading
	};
}

// add a different type for each reading
namespace DataTypes {
    enum DataType {
		Temperature = 0,    // float
		Pressure = 1,       // long
		LocationData = 2,
		PressureBMP = 3,	// long
		Humidity = 4,       // float
		ErrorInfo = 5,
        Counter,            // int
		TemperatureMS,      // double
		TemperatureTMP,     // float
		PMS5003,
		MPU6050,
		Timestamp,
		TemperatureBMP,		// float
    };
}

namespace DataQueue {

    // add a new type with name typeValue when a new type is needed
    // these types can be reused with different DataTypes
    union DataUnion {
        int intValue;
        float floatValue;
        double doubleValue;
        long longValue;
        PMSData pmsValue;
        MPUData mpuData;
        LocationData locationData;
        ErrorTypes::ErrorType errorInfo;
    };

    // queues store this struct
    struct QueueElement {
        DataTypes::DataType type; // what are we storing
        DataUnion data;           // actual data
        uint16_t time;			  // seconds from startup (rollover after 18h)
    };

    // used to pass parameters to flushFunction as void*
    struct FlushFunctionParams {
        int flush_delay;                          // how often to flush
        void (*dataParser)(QueueHandle_t queue);  // function parsing queue elements
        QueueHandle_t *queue;                     // queue pointer
    };

    // task for flushing the queue
    void flushFunction(void *pvParameters) {
        // unpack parameters
        FlushFunctionParams params = *((FlushFunctionParams*) pvParameters);

        TickType_t xLastWakeTime;
        while (true) {
            xLastWakeTime = xTaskGetTickCount();

            QueueElement element;

			// tell parser to read all elements from queue
            params.dataParser(*(params.queue));

            // call parser for all elements
            /*while (xQueueReceive(*(params.queue), &element, 0)) {
                params.dataParser(element);
            }*/

			// debugD("%d free space", uxTaskGetStackHighWaterMark(NULL));

            // sleep
			debugD("Sleep for %d", params.flush_delay);
            vTaskDelayUntil(&xLastWakeTime, params.flush_delay / portTICK_PERIOD_MS);
            // vTaskDelay(params.flush_delay / portTICK_PERIOD_MS);
        }
        vTaskDelete(NULL);
    }

    // main queue class
    class Queue {
    private:
        QueueHandle_t m_queue;
        TaskHandle_t m_flushFunction = NULL;
		FlushFunctionParams m_params;

    public:
        Queue() {
            // create queue storing QueueElements
            // m_queue = xQueueCreate(size, sizeof(QueueElement));
        }

		void begin(int size) {
			m_queue = xQueueCreate(size, sizeof(QueueElement));
		}

        ~Queue() {
        	if (m_flushFunction != NULL)
        		vTaskDelete(m_flushFunction);
        	vQueueDelete(m_queue);
        }

        // used to start flushing queue with a parser
        void setFlushFunction(void (*dataParser)(QueueHandle_t queue),
                              int flush_delay, const char *const pcName, int priority) {
            // if not flushing already
            if (m_flushFunction == NULL) {
                // pack params
                m_params = FlushFunctionParams {
                    flush_delay,
                    dataParser,
                    &m_queue
                };

                // schedule flushing task
                xTaskCreate(flushFunction, pcName, 16*1024, (void*)&m_params, priority, &m_flushFunction);
            }
        }

        // for adding data to queue
        BaseType_t add(QueueElement *data, bool isr = false, BaseType_t *pxTaskWoken = NULL) {
            if (m_queue != 0) {
                // handle calls from interrupts (ISR)
                if (isr == false) {
                                                 /* wait 50ms if queue is blocked */
                    return xQueueSend(m_queue, data, 50 / portTICK_PERIOD_MS);
                } else {
                    return xQueueSendFromISR(m_queue, data, pxTaskWoken);
                }
            }
            return errQUEUE_BLOCKED;
        }
    };

} // namespace DataQueue

// creates queue element from error code
DataQueue::QueueElement ErrorTypeToElement(ErrorTypes::ErrorType error) {
	DataQueue::DataUnion data;
	data.errorInfo = error;

	DataQueue::QueueElement element = {
		.type = DataTypes::ErrorInfo,
		.data = data,
		.time = (uint16_t) (millis() / 1000)
	};

	return element;
}

// converts queue element to json format
void elementToJson(DataQueue::QueueElement element, char *target) {
	char temp[256] = {'\0'};
	char buf[16] = {'\0'};
	char time_buf[32] = {'\0'};

	char lat_buf[16] = {'\0'};
	char lng_buf[16] = {'\0'};
	char alt_buf[16] = {'\0'};

	// add timestamp
	strcat(target, R"({"time":)");
	sprintf(time_buf, "%u,", element.time);
	strcat(target, time_buf);

	switch (element.type) {
		case DataTypes::Counter:
			// add type and value
			strcat(target, R"("type":"Counter",)");
			sprintf(temp, R"("value":%d})",
					element.data.intValue);
			strcat(target, temp);
			break;

		case DataTypes::Temperature:
			dtostrf(element.data.floatValue, 3, 2, buf);

			strcat(target, R"("type":"Temperature",)");
			sprintf(temp, R"("value":%s})",
					buf);
			strcat(target, temp);
			break;

		case DataTypes::TemperatureMS:
			dtostrf(element.data.doubleValue, 3, 2, buf);

			strcat(target, R"("type":"TemperatureMS",)");
			sprintf(temp, R"("value":%s})",
					buf);
			strcat(target, temp);
			break;

		case DataTypes::TemperatureTMP:
			dtostrf(element.data.floatValue, 3, 2, buf);

			strcat(target, R"("type":"TemperatureTMP",)");
			sprintf(temp, R"("value":%s})",
					buf);
			strcat(target, temp);
			break;

		case DataTypes::Pressure:
			strcat(target, R"("type":"Pressure",)");
			sprintf(temp, R"("value":%ld})",
					element.data.longValue);
			strcat(target, temp);
			break;

		case DataTypes::Humidity:
			dtostrf(element.data.floatValue, 3, 2, buf);

			strcat(target, R"("type":"Humidity",)");
			sprintf(temp, R"("value":%s})",
					buf);
			strcat(target, temp);
			break;

		case DataTypes::MPU6050:
			// strcat(target, R"("type":"MPU6050",)");
			break;

		case DataTypes::PMS5003:
			strcat(target, R"("type":"PMS",)");
			sprintf(temp, R"("value":[%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u]})",
					element.data.pmsValue.data1,
					element.data.pmsValue.data2,
					element.data.pmsValue.data3,
					element.data.pmsValue.data4,

					element.data.pmsValue.data5,
					element.data.pmsValue.data6,
					element.data.pmsValue.data7,
					element.data.pmsValue.data8,

					element.data.pmsValue.data9,
					element.data.pmsValue.data10,
					element.data.pmsValue.data11,
					element.data.pmsValue.data12,

					element.data.pmsValue.data13);
			strcat(target, temp);
			break;

		case DataTypes::LocationData:
			strcat(target, R"("type":"Location",)");

			dtostrf(element.data.locationData.lat, 3, 4, lat_buf);
			dtostrf(element.data.locationData.lng, 3, 4, lng_buf);
			dtostrf(element.data.locationData.alt, 3, 4, alt_buf);
			
			sprintf(temp, R"("value":{"lat":%s,"lng":%s,"alt":%s}})",
					lat_buf, lng_buf, alt_buf);
			strcat(target, temp);
			break;

		default:
			break;

		case DataTypes::TemperatureBMP:
			dtostrf(element.data.floatValue, 3, 2, buf);

			strcat(target, R"("type":"TemperatureBMP",)");
			sprintf(temp, R"("value":%s})",
					buf);
			strcat(target, temp);
			break;

		case DataTypes::PressureBMP:
			strcat(target, R"("type":"PressureBMP",)");
			sprintf(temp, R"("value":%ld})",
					element.data.longValue);
			strcat(target, temp);
			break;
	}
}

// print queue element (called from interrupts)
void logElementAsync(DataQueue::QueueElement element) {
	switch (element.type) {
		case DataTypes::Counter:
			Serial.print("Counter = ");
			Serial.println(element.data.intValue);
			break;

		case DataTypes::Temperature:
			Serial.print("Temperature = ");
			Serial.println(element.data.floatValue);
			break;

		case DataTypes::TemperatureMS:
			Serial.print("TemperatureMS = ");
			Serial.println(element.data.doubleValue);
			break;

		case DataTypes::TemperatureTMP:
			Serial.print("TemperatureTMP = ");
			Serial.println(element.data.floatValue);
			break;

		case DataTypes::Pressure:
			Serial.print("Pressure = ");
			Serial.println(element.data.longValue);
			break;

		case DataTypes::Humidity:
			Serial.print("Humidity = ");
			Serial.println(element.data.floatValue);
			break;

		case DataTypes::PMS5003:
			Serial.print("PMS2 = ");
			Serial.println(element.data.pmsValue.data2);
			Serial.print("PMS3 = ");
			Serial.println(element.data.pmsValue.data3);
			break;

		case DataTypes::MPU6050:
			Serial.print("MPUpitch = ");
			Serial.println(element.data.mpuData.pitch);
			Serial.print("MPUroll = ");
			Serial.println(element.data.mpuData.roll);
			break;

		case DataTypes::LocationData:
			Serial.print("Lat = ");
			Serial.print(element.data.locationData.lat);
			Serial.print(" Lng = ");
			Serial.print(element.data.locationData.lng);
			Serial.print(" Alt = ");
			Serial.println(element.data.locationData.alt);

			{
				double distance2Sender = TinyGPSPlus::distanceBetween(element.data.locationData.lat,
																	  element.data.locationData.lng,
																	  50.088651,
																	  19.813234);
				Serial.print("Distance to sender = ");
				Serial.print(distance2Sender);
				Serial.println("m");
			}

			break;

		case DataTypes::ErrorInfo:
			/*
			switch (element.data.errorInfo) {
				case ErrorTypes::I2CBlocked:
					debugE("I2C blocked");
					break;

				case ErrorTypes::SemaphoreNULL:
					debugE("Semaphore NULL");
					break;

				case ErrorTypes::LoraBlocked:
					debugE("Lora blocked");
					break;

				case ErrorTypes::MSNotStarted:
					debugE("MS not started");
					break;

				case ErrorTypes::TMPNotStarted:
					debugE("TMP not started");
					break;
			}
			*/
			break;

		case DataTypes::TemperatureBMP:
			Serial.print("TemperatureBMP = ");
			Serial.println(element.data.floatValue);
			break;

		case DataTypes::PressureBMP:
			Serial.print("PressureBMP = ");
			Serial.println(element.data.longValue);
			break;

		default:
			Serial.print("Unknown data type\n");
			break;
	}
}

// prints element (called not from interrupt)
void logElement(DataQueue::QueueElement element, bool async = false) {
	if (async) {
		logElementAsync(element);
		return;
	}

	switch (element.type) {
		case DataTypes::Counter:
			printf("Counter = %d\n", element.data.intValue);
			debugI("Counter = %d\n", element.data.intValue);
			break;

		case DataTypes::Temperature:
			printf("Temperature = %g celsius\n", element.data.floatValue);
			debugI("Temperature = %g celsius\n", element.data.floatValue);
			break;

		case DataTypes::TemperatureMS:
			printf("TemperatureMS = %g celsius\n", element.data.doubleValue);
			debugI("TemperatureMS = %g celsius\n", element.data.doubleValue);
			break;

		case DataTypes::TemperatureTMP:
			printf("TemperatureTMP = %g celsius\n", element.data.floatValue);
			debugI("TemperatureTMP = %g celsius\n", element.data.floatValue);
			break;

		case DataTypes::Pressure:
			printf("Pressure = %ld\n", element.data.longValue);
			debugI("Pressure = %ld\n", element.data.longValue);
			break;

		case DataTypes::Humidity:
			printf("Humidity = %g%%\n", element.data.floatValue);
			debugI("Humidity = %g%%\n", element.data.floatValue);
			break;

		case DataTypes::PMS5003:
			printf("Data1: %u, Data2: %u, Data3: %u\nData4: %u, Data5: %u, Data6: %u\n"
				   "Data7: %u, Data8: %u, Data9: %u\nData10: %u, Data11: %u, Data12: %u\n"
				   "Data13: %u\n",
					element.data.pmsValue.data1,
					element.data.pmsValue.data2,
					element.data.pmsValue.data3,
					element.data.pmsValue.data4,

					element.data.pmsValue.data5,
					element.data.pmsValue.data6,
					element.data.pmsValue.data7,
					element.data.pmsValue.data8,

					element.data.pmsValue.data9,
					element.data.pmsValue.data10,
					element.data.pmsValue.data11,
					element.data.pmsValue.data12,

					element.data.pmsValue.data13);
			debugI("Data1: %u, Data2: %u, Data3: %u\nData4: %u, Data5: %u, Data6: %u\n"
				   "Data7: %u, Data8: %u, Data9: %u\nData10: %u, Data11: %u, Data12: %u\n"
				   "Data13: %u\n",
					element.data.pmsValue.data1,
					element.data.pmsValue.data2,
					element.data.pmsValue.data3,
					element.data.pmsValue.data4,

					element.data.pmsValue.data5,
					element.data.pmsValue.data6,
					element.data.pmsValue.data7,
					element.data.pmsValue.data8,

					element.data.pmsValue.data9,
					element.data.pmsValue.data10,
					element.data.pmsValue.data11,
					element.data.pmsValue.data12,

					element.data.pmsValue.data13);
			break;

		case DataTypes::MPU6050:
			printf("Roll = %g\tPitch = %g\tYaw = %g\n"
					"aX = %g\taY = %g\taZ = %g\n",
					element.data.mpuData.roll,
					element.data.mpuData.pitch,
					element.data.mpuData.yaw,

					element.data.mpuData.ax,
					element.data.mpuData.ay,
					element.data.mpuData.az);
			debugI("Roll = %g\tPitch = %g\tYaw = %g\n"
					"aX = %g\taY = %g\taZ = %g\n",
					element.data.mpuData.roll,
					element.data.mpuData.pitch,
					element.data.mpuData.yaw,

					element.data.mpuData.ax,
					element.data.mpuData.ay,
					element.data.mpuData.az);
			break;

		case DataTypes::LocationData:
			printf("Lat = %f Lng = %f Alt = %u",
					element.data.locationData.lat,
					element.data.locationData.lng,
					element.data.locationData.alt);
			debugI("Lat = %f Lng = %f Alt = %u",
					element.data.locationData.lat,
					element.data.locationData.lng,
					element.data.locationData.alt);
			{
			double dist2Carbon = TinyGPSPlus::distanceBetween(element.data.locationData.lat,
										 	 	 	 	 	  element.data.locationData.lng,
															  50.092438,
															  19.818403);
			printf("Distance to Carbon = %gm\n", dist2Carbon);
			debugW("Distance to Carbon = %gm", dist2Carbon);
			}

			break;

		case DataTypes::ErrorInfo:
			/*
			switch (element.data.errorInfo) {
				case ErrorTypes::I2CBlocked:
					debugE("I2C blocked");
					break;

				case ErrorTypes::SemaphoreNULL:
					debugE("Semaphore NULL");
					break;

				case ErrorTypes::LoraBlocked:
					debugE("Lora blocked");
					break;

				case ErrorTypes::MSNotStarted:
					debugE("MS not started");
					break;

				case ErrorTypes::TMPNotStarted:
					debugE("TMP not started");
					break;
			}
			*/
			break;

		case DataTypes::TemperatureBMP:
			printf("TemperatureBMP = %g celsius\n", element.data.floatValue);
			debugI("TemperatureBMP = %g celsius\n", element.data.floatValue);
			break;

		case DataTypes::PressureBMP:
			printf("PressureBMP = %ld\n", element.data.longValue);
			debugI("PressureBMP = %ld\n", element.data.longValue);
			break;

		default:
			printf("Unknown data type\n");
			debugE("Unknown data type");
			break;
	}
}

#endif
