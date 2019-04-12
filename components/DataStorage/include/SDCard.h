#ifndef _SDCARD_H_
#define _SDCARD_H_

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "FS.h"
#include "SD.h"
#include "SPI.h"

#include "DataQueue.h"
#include "FSFunctions.h"
#include "OTAService.h"
#include "RemoteDebug.h"

extern RemoteDebug Debug;


namespace SDCard {

	const char *m_TAG = "SD Card";

	bool m_mounted = false;

    void begin() {
    	if(!SD.begin()){
			ESP_LOGE(m_TAG, "Card Mount Failed");
			debugE("Card Mount Failed");
			return;
		}

    	m_mounted = true;

    	uint8_t cardType = SD.cardType();

		if(cardType == CARD_NONE){
			ESP_LOGE(m_TAG, "No SD card attached");
			debugE("No SD card attached");
			return;
		}

		ESP_LOGD(m_TAG, "SD Card Type: %s", [cardType]() {
			if(cardType == CARD_MMC){
				return "MMC";
			} else if(cardType == CARD_SD){
				return "SDSC";
			} else if(cardType == CARD_SDHC){
				return "SDHC";
			} else {
				return "UNKNOWN";
			}
		}());

		debugD("SD Card Type: %s", [cardType]() {
			if(cardType == CARD_MMC){
				return "MMC";
			} else if(cardType == CARD_SD){
				return "SDSC";
			} else if(cardType == CARD_SDHC){
				return "SDHC";
			} else {
				return "UNKNOWN";
			}
		}());

		uint64_t cardSize = SD.cardSize() / (1024 * 1024);
		ESP_LOGD(m_TAG, "SD Card Size: %lluMB", cardSize);
		debugD("SD Card Size: %lluMB", cardSize);

		ESP_LOGD(m_TAG, "Total space: %lluMB", SD.totalBytes() / (1024 * 1024));
		ESP_LOGD(m_TAG, "Used space: %lluMB", SD.usedBytes() / (1024 * 1024));

		debugD("Total space: %lluMB", SD.totalBytes() / (1024 * 1024));
		debugD("Used space: %lluMB", SD.usedBytes() / (1024 * 1024));

		ESP_LOGI(m_TAG, "SD Card started");
		debugI("SD Card started");


		ESP_LOGI(m_TAG, "Testing SD");
		debugI("Testing SD");
		FSFunctions::createDir(SD, "/test");
		FSFunctions::listDir(SD, "/", 0);
		FSFunctions::writeFile(SD, "/hello.txt", "Hello ");
		FSFunctions::appendFile(SD, "/hello.txt", "World!\r\n");
		FSFunctions::readFile(SD, "/hello.txt");
		FSFunctions::renameFile(SD, "/hello.txt", "/foo.txt");
		FSFunctions::readFile(SD, "/foo.txt");
		FSFunctions::listDir(SD, "/", 0);
		// FSFunctions::deleteFile(SD, "/foo.txt");
		// FSFunctions::testFileIO(SD, "/test.txt");
		// FSFunctions::deleteFile(SD, "/test.txt");
		ESP_LOGI(m_TAG, "Test complete");
		debugI("Test complete");

    }

    void saveString(const char *data, const char *name, const char *flightName) {
    	char path[32] = {'\0'};

    	/*sprintf(path, "/%s", flightName);
    	if (!SD.exists(path)) {
    		FSFunctions::createDir(SD, path);
    	}

		strcat(path, "/");
		strcat(path, name);
		strcat(path, ".json");*/

    	sprintf(path, "/%s", name);

		// debugI("%s -> %s", data, path);

		if (!SD.exists(path)) {
			debugI("Creating %s", path);
			FSFunctions::writeFile(SD, path, "\n\n");
		}

		FSFunctions::appendFile(SD, path, data);
    }

    void saveElement(DataQueue::QueueElement element) {

    }

};

#endif
