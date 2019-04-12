#ifndef _DATASTORAGE_H_
#define _DATASTORAGE_H_

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "SDCard.h"
#include "SPIFFSHAL.h"
#include "DataQueue.h"
#include "RemoteDebug.h"

extern RemoteDebug Debug;

namespace DataStorage {

	const char *m_TAG = "DataStorage";

	char m_flightName[9] = {'\0'};

    void begin() {
    	// vTaskDelay(5000 / portTICK_PERIOD_MS);
    	ESP_LOGI(m_TAG, "Staring data storage");
    	debugI("Staring data storage");

    	strcpy(m_flightName, "test");

    	SDCard::begin();
    	SPIFFSHAL::begin();
    }

    void setFlightName(const char *flightName) {
    	strcpy(m_flightName, flightName);
    }

    void saveElement(DataQueue::QueueElement element) {
    	debugI("Saving element");
    	char jsonObject[256] = {'\0'};

    	elementToJson(element, jsonObject);
    	strcat(jsonObject, ",\n");

    	switch (element.type) {
			case DataTypes::Counter:

				break;

			case DataTypes::Temperature:
				SDCard::saveString(jsonObject, "dht", m_flightName);
				break;

			case DataTypes::TemperatureMS:
				SDCard::saveString(jsonObject, "mst", m_flightName);
				break;

			case DataTypes::TemperatureTMP:
				SDCard::saveString(jsonObject, "tmp", m_flightName);
				break;

			case DataTypes::Pressure:
				SDCard::saveString(jsonObject, "pres", m_flightName);
				break;

			case DataTypes::Humidity:
				SDCard::saveString(jsonObject, "hum", m_flightName);
				break;

			case DataTypes::MPU6050:

				break;

			case DataTypes::PMS5003:
				SDCard::saveString(jsonObject, "pms", m_flightName);
				break;

			case DataTypes::LocationData:
				SDCard::saveString(jsonObject, "gps", m_flightName);
				break;

			default:
				break;
    	}
    }

};

#endif
