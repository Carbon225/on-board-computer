#ifndef _OTASERVICE_H_
#define _OTASERVICE_H_

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"

#include "WiFi.h"
#include "WiFiClient.h"
#include "WebServer.h"
#include "ESPmDNS.h"
#include "Update.h"
#include "../webpage.h"

#include "RemoteDebug.h"

RemoteDebug Debug;

namespace OTAService {

	// const char* host = "cansat-esp32";
	const char* ssid = "xtr-cansat";
	const char* password = "12345osiem";

	/*const char* ssid = "Carbon Industries Server Farm";
	const char* password = "****";*/

	WebServer server(80);
	WiFiClient client;

	bool debug_started = false;

	void loop();

    void OTATask(void*) {
        TickType_t xLastWakeTime;
        for (;;) {
            // xLastWakeTime = xTaskGetTickCount();
            loop();
            vTaskDelay(1 / portTICK_PERIOD_MS);
        }
        vTaskDelete(NULL);
    }

    void testBegin(const char *host) {
    	// Connect to WiFi network
		WiFi.begin(ssid, password);
		Serial.println("");

		// Wait for connection
		while (WiFi.status() != WL_CONNECTED) {
			delay(500);
			Serial.print(".");
		}
		Serial.println("");
		Serial.print("Connected to ");
		Serial.println(ssid);
		Serial.print("IP address: ");
		Serial.println(WiFi.localIP());

		/*use mdns for host name resolution*/
		if (!MDNS.begin(host)) { //http://esp32.local
			Serial.println("Error setting up MDNS responder!");
			while (1) {
				delay(1000);
			}
		}

		MDNS.addService("telnet", "tcp", 23);

		Serial.println("mDNS responder started");

		Debug.begin(host); // Initiaze the telnet server
		Debug.setResetCmdEnabled(true); // Enable the reset command
		Debug.showProfiler(true); // Profiler (Good to measure times, to optimize codes)
		Debug.showColors(true); // Colors

		debug_started = true;

		/*return index page which is stored in serverIndex */
		server.on("/", HTTP_GET, []() {
			server.sendHeader("Connection", "close");
			server.send(200, "text/html", loginIndex);
		});

		server.on("/serverIndex", HTTP_GET, []() {
			server.sendHeader("Connection", "close");
			server.send(200, "text/html", serverIndex);
		});

		/*handling uploading firmware file */
		server.on("/update", HTTP_POST, []() {
			server.sendHeader("Connection", "close");
			server.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
			ESP.restart();
		}, []() {
			HTTPUpload& upload = server.upload();
			if (upload.status == UPLOAD_FILE_START) {
				debugI("Downloading OTA");
				Serial.printf("Update: %s\n", upload.filename.c_str());
				if (!Update.begin(UPDATE_SIZE_UNKNOWN)) { //start with max available size
					Update.printError(Serial);
				}
			} else if (upload.status == UPLOAD_FILE_WRITE) {
				/* flashing firmware to ESP*/
				if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
					Update.printError(Serial);
				}
			} else if (upload.status == UPLOAD_FILE_END) {
				if (Update.end(true)) { //true to set the size to the current progress
					debugI("OTA successful, rebooting");
					Serial.printf("Update Success: %u\nRebooting...\n", upload.totalSize);
				} else {
					Update.printError(Serial);
				}
			}
		});

		server.begin();
        xTaskCreate(OTATask, "OTATask", 1024*8, NULL, 2, NULL);
    }

    void beginBasic(const char *host) {
    	// Connect to WiFi network
		WiFi.begin(ssid, password);
		Serial.println("");

		// Wait for connection
		while (WiFi.status() != WL_CONNECTED) {
			delay(500);
			Serial.print(".");
		}
		Serial.println("");
		Serial.print("Connected to ");
		Serial.println(ssid);
		Serial.print("IP address: ");
		Serial.println(WiFi.localIP());

		/*use mdns for host name resolution*/
		if (!MDNS.begin(host)) { //http://esp32.local
			Serial.println("Error setting up MDNS responder!");
			while (1) {
				delay(1000);
			}
		}

		Serial.println("mDNS responder started");

		/*return index page which is stored in serverIndex */
		server.on("/", HTTP_GET, []() {
			server.sendHeader("Connection", "close");
			server.send(200, "text/html", loginIndex);
		});

		server.on("/serverIndex", HTTP_GET, []() {
			server.sendHeader("Connection", "close");
			server.send(200, "text/html", serverIndex);
		});

		/*handling uploading firmware file */
		server.on("/update", HTTP_POST, []() {
			server.sendHeader("Connection", "close");
			server.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
			ESP.restart();
		}, []() {
			HTTPUpload& upload = server.upload();
			if (upload.status == UPLOAD_FILE_START) {
				debugI("Downloading OTA");
				Serial.printf("Update: %s\n", upload.filename.c_str());
				if (!Update.begin(UPDATE_SIZE_UNKNOWN)) { //start with max available size
					Update.printError(Serial);
				}
			} else if (upload.status == UPLOAD_FILE_WRITE) {
				/* flashing firmware to ESP*/
				if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
					Update.printError(Serial);
				}
			} else if (upload.status == UPLOAD_FILE_END) {
				if (Update.end(true)) { //true to set the size to the current progress
					debugI("OTA successful, rebooting");
					Serial.printf("Update Success: %u\nRebooting...\n", upload.totalSize);
				} else {
					Update.printError(Serial);
				}
			}
		});

		server.begin();
        xTaskCreate(OTATask, "OTATask", 1024*8, NULL, 2, NULL);
    }

    void loop() {
    	server.handleClient();
    	Debug.handle();
    }

}

#endif
