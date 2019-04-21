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

    bool m_started = false;

    // name for ESP_LOG
    const char *m_TAG = "DataStorage";

    void begin() {
        // vTaskDelay(5000 / portTICK_PERIOD_MS);
        ESP_LOGI(m_TAG, "Staring data storage");
        debugI("Staring data storage");

        SDCard::begin();
        // SPIFFSHAL::begin();

        m_started = true;
    }

    void saveElement(DataQueue::QueueElement element) {
        if (!m_started)
            return;

        debugV("Saving element");
        char jsonObject[256] = {'\0'};

        // convert element to json format
        elementToJson(element, jsonObject);
        strcat(jsonObject, ",\n");

        // based on type save to SD card or SPIFFS
        switch (element.type) {
            case DataTypes::Counter:

                break;

            case DataTypes::Temperature:
                SDCard::saveString(jsonObject, "dht");
                break;

            case DataTypes::TemperatureMS:
                SDCard::saveString(jsonObject, "mst");
                break;

            case DataTypes::TemperatureTMP:
                SDCard::saveString(jsonObject, "tmp");
                break;

            case DataTypes::Pressure:
                SDCard::saveString(jsonObject, "pres");
                break;

            case DataTypes::Humidity:
                SDCard::saveString(jsonObject, "hum");
                break;

            case DataTypes::PMS5003:
                SDCard::saveString(jsonObject, "pms");
                break;

            case DataTypes::LocationData:
                SDCard::saveString(jsonObject, "gps");
                break;

            default:
                break;
        }
    }

};

#endif
