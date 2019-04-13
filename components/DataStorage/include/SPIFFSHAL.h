#ifndef _SPIFFSHAL_H_
#define _SPIFFSHAL_H_

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "FS.h"
#include "SPIFFS.h"
#include "FSFunctions.h"
#include "DataQueue.h"
#include "RemoteDebug.h"

extern RemoteDebug Debug;

#define FORMAT_SPIFFS_IF_FAILED false

namespace SPIFFSHAL {

	const char *m_TAG = "SPIFFS";

	bool m_mounted = false;

    void begin() {
    	if(!SPIFFS.begin(FORMAT_SPIFFS_IF_FAILED)){
			ESP_LOGE(m_TAG, "SPIFFS Mount Failed");
			debugE("SPIFFS Mount Failed");
			return;
		}

		/*
    	ESP_LOGI(m_TAG, "Testing SPIFFS");
    	debugI("Testing SPIFFS");
    	FSFunctions::listDir(SPIFFS, "/", 0);
    	FSFunctions::writeFile(SPIFFS, "/hello.txt", "Hello ");
    	FSFunctions::appendFile(SPIFFS, "/hello.txt", "World!\r\n");
    	FSFunctions::readFile(SPIFFS, "/hello.txt");
    	FSFunctions::renameFile(SPIFFS, "/hello.txt", "/foo.txt");
    	FSFunctions::readFile(SPIFFS, "/foo.txt");
    	FSFunctions::listDir(SPIFFS, "/", 0);
    	// FSFunctions::deleteFile(SPIFFS, "/foo.txt");
    	// FSFunctions::testFileIO(SPIFFS, "/test.txt");
    	// FSFunctions::deleteFile(SPIFFS, "/test.txt");
    	ESP_LOGI(m_TAG, "Test complete");
    	debugI("Test complete");
		*/
    }

    void saveString(const char *data, const char *name) {
		char path[16] = {'\0'};

		sprintf(path, "/%s", name);

		if (!SPIFFS.exists(path)) {
			debugI("Creating %s", path);
			FSFunctions::writeFile(SPIFFS, path, "\n\n");
		}

		FSFunctions::appendFile(SPIFFS, path, data);
	}

};

#endif
