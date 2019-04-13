#ifndef _RADIOHAL_H_
#define _RADIOHAL_H_

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"

#include "Arduino.h"
#include "SPI.h"
#include "LoRa.h"
#include "DataQueue.h"
#include "Decoder.h"
#include "RemoteDebug.h"

extern RemoteDebug Debug;
extern SemaphoreHandle_t lora_mutex;
extern DataQueue::Queue sendQueue;


class RadioHAL {
private:
	const char *m_TAG = "RadioHAL";
	void (*m_onReceive)(int packetSize) = 0;
	bool _started = false;

public:
	RadioHAL(int cs, int rst, int dio0) {
		LoRa.setPins(cs, rst, dio0);
	}

	void begin(long freq) {
		ESP_LOGI(m_TAG, "Starting radio");
		if (!LoRa.begin(freq)) {
			ESP_LOGE(m_TAG, "Starting LoRa failed!");
		} else {
			

			int txPower = 20;
			int sf = 8;
			// long sbw = 62.5E3;
			long sbw = 31.25E3;
			// long sbw = 125E3;
			int crd = 8;

			// set radio parameters
			LoRa.setTxPower(txPower);
			LoRa.setSpreadingFactor(sf);
			LoRa.setSignalBandwidth(sbw);
			LoRa.setCodingRate4(crd);
			
			_started = true;
			ESP_LOGI(m_TAG, "Radio started");
			Serial.print("TX power = ");
			Serial.println(txPower);

			Serial.print("SF = ");
			Serial.println(sf);
			
			Serial.print("SBW = ");
			Serial.println(sbw / 1000);
			
			Serial.print("CR = 4/");
			Serial.println(crd);
			
			debugI("Radio started");
			debugD("TX power = %d\nSF = %d\nSBW = %d\nCR = 4/%d",
					txPower, sf, (int) (sbw / 1000), crd);
		}
	}

	// start receiving data
	void startReceive(void (*onReceive)(int packetSize), int packetSize = 0) {
		m_onReceive = onReceive;
		LoRa.onReceive(m_onReceive);
		LoRa.receive(packetSize);
	}

	void send(uint8_t *data) {
		if (_started) {
			if (lora_mutex != NULL) {
				// get lora semaphore
				if (xSemaphoreTake(lora_mutex, 20 / portTICK_PERIOD_MS)) {
					ESP_LOGD(m_TAG, "Sending packet");

					TickType_t function_start = xTaskGetTickCount();

					while (!LoRa.beginPacket(true)) {
						vTaskDelay(10 / portTICK_PERIOD_MS);
					}

					TickType_t transmission_start = xTaskGetTickCount();

					LoRa.write(data, PACKET_SIZE);
					LoRa.endPacket(false); // async

					xSemaphoreGive(lora_mutex);

					int send_delay = (transmission_start - function_start) * portTICK_PERIOD_MS;
					int transmission_time = (xTaskGetTickCount() - transmission_start) * portTICK_PERIOD_MS;
					ESP_LOGI(m_TAG, "Packet sent in %d ms, late by %d ms", transmission_time, send_delay);
					debugD("Packet sent in %d ms, late by %d ms", transmission_time, send_delay);
				} else {
					DataQueue::QueueElement error = ErrorTypeToElement(ErrorTypes::LoraBlocked);
					sendQueue.add(&error);
				}
			} else {
				DataQueue::QueueElement error = ErrorTypeToElement(ErrorTypes::SemaphoreNULL);
				sendQueue.add(&error);
			}
		}
	}
};


#endif
