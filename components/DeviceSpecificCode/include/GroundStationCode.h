#ifndef _GROUNDSTATIONCODE_H_
#define _GROUNDSTATIONCODE_H_

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "Arduino.h"
#include "DataQueue.h"
#include "RemoteDebug.h"

extern RemoteDebug Debug;


namespace GroundStation {

	// interrupt
	void onReceive(int packetSize) {
		if (packetSize == PACKET_SIZE)
		{
			Serial.println("");
			Serial.println("Receiving packet...");

			Serial.print("RSSI = ");
			Serial.println(LoRa.packetRssi());

			Serial.print("SNR = ");
			Serial.println(LoRa.packetSnr());

			uint8_t data[PACKET_SIZE];
			// receive packet from probe
			LoRa.readBytes(data, PACKET_SIZE);
			// generateSampleData(data);

			// parse every element
			int ret = parseData(data, [](DataQueue::QueueElement element) {
				// onReceive is an interrupt so we use async function
				logElementAsync(element);
			});

			switch (ret)
			{
			case 1:
				Serial.println("ERROR invalid packet");
				break;
			case 2:
				Serial.println("ERROR invalid data type");
			}
		}
		else
		{
			Serial.println("ERROR wrong packet size");
		}
	}

	// serial for sending data to computer
	void beginSerial() {
		Serial2.begin(115200);
	}

	// send 1 element to computer
	void sendElement(DataQueue::QueueElement element) {
		char message[256] = {'\0'};
		elementToJson(element, message);
		Serial2.println(message);
	}

	// send array of elements
	void sendArray(DataQueue::QueueElement elements[], int length) {
		for (int i = 0; i < length; i++) {
			sendElement(elements[i]);
		}
	}

} // namespace GroundStation


#endif /* _GROUNDSTATIONCODE_H_ */
