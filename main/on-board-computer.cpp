#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/gpio.h"

// #define ENABLE_WIFI

#include "Arduino.h"
#include "Wire.h"
#include "DataQueue.h"

#include "DHT22Sensor.h"
#include "CounterSensor.h"
#include "PMS5003Sensor.h"
#include "GPSSensor.h"
#include "MS5611Sensor.h"
#include "TMP102Sensor.h"
#include "BMP180Sensor.h"

#include "RemoteDebug.h"
#include "DataStorage.h"
#include "RadioHAL.h"
#include "OTAService.h"
#include "Decoder.h"


#define SAFE_MODE_PIN 13

DHT22Sensor dht22("dht"); // (GPIO_NUM_4, "DHTn1");
PMS5003Sensor pms5003(UART_NUM_2, "pms");
RadioHAL radio;
MS5611Sensor ms5611("ms");
TMP102Sensor tmp102("tmp");
GPSSensor gps("gps");
BMP180Sensor bmp180("bmp");

// mutex semaphore to make sure only one task at a time can use i2c and radio
SemaphoreHandle_t i2c_mutex = NULL;
SemaphoreHandle_t lora_mutex = NULL;

// main data queue for sending data over radio
DataQueue::Queue sendQueue;
// queue for storing data on SD card
DataQueue::Queue saveQueue;

int counter = 0;

// test task to see if device works
void loopTask(void *ignore) {
    for (;;) {
        DataQueue::DataUnion data;
        data.intValue = counter;

        DataQueue::QueueElement element = {
            .type = DataTypes::Counter,
            .data = data,
			.time = (uint16_t) (millis() / 1000)
        };

        sendQueue.add(&element);
        counter++;

        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

// used to parse queue elements and compose radio packets
void queueDataParser(QueueHandle_t queue) {
	DataQueue::QueueElement element;
	uint8_t packet[PACKET_SIZE + sizeof(DataQueue::QueueElement)] = {0}; // overhead for last element
	int packet_size = 1;

	// peek and encode time from first element
	xQueuePeek(queue, &element, 0);
	memcpy((void*)(packet + packet_size), (void*)&element.time, sizeof(element.time));
	packet_size += sizeof(element.time);

	debugD("Time = %u Encoded = %u", element.time, *(uint16_t*)(packet + 1));

	// read next element
	while (xQueuePeek(queue, &element, 0)) {
		// don't send data other than temperature, pressure and location
		if (element.type != DataTypes::Temperature &&
			element.type != DataTypes::Pressure &&
			element.type != DataTypes::LocationData &&
			element.type != DataTypes::PressureBMP) {
				// clear element from queue
				xQueueReceive(queue, &element, 0);
				// read next element
				continue;
			}
		logElement(element);

		// try to encode and get size of encoded data
		int new_element_size = encode(element, packet + packet_size);

		// if no space left in packet stop encoding
		if (packet_size + new_element_size > PACKET_SIZE - 2) {
			debugW("Packet filled");
			break; // element was peeked so it will remain in queue
		}
		// if encoding finished set new packet size
		packet_size += new_element_size;

		// clear element from queue
		xQueueReceive(queue, &element, 0);
	}

	// fill with 0xff
	for (int i = packet_size; i < PACKET_SIZE; i++) {
		packet[i] = 0xff;
	}

	// generate packet verify
	uint8_t sum = 0;
	for (int i = 0; i < PACKET_SIZE - 1; i++) {
		sum += packet[i];
	}
	packet[PACKET_SIZE - 1] = sum;

	if (packet_size > 1)
		radio.send(packet);
}

void saveDataParser(QueueHandle_t queue) {
	DataQueue::QueueElement element;

	// read next element
	while (xQueueReceive(queue, &element, 0)) {
		debugI("Saving element %u:", element.type);
		logElement(element);
		// save element to storage
		DataStorage::saveElement(element);
	}
}

int core0idle_count = 0;
int core1idle_count = 0;

void core0Idle(void*) {
	while (true) {
		core0idle_count++;
		vTaskDelay(5 / portTICK_PERIOD_MS);
	}
}

void core1Idle(void*) {
	while (true) {
		core1idle_count++;
		vTaskDelay(5 / portTICK_PERIOD_MS);
	}
}

void clearCount(void*) {
	TickType_t xLastWakeTime;
        while (true) {
            xLastWakeTime = xTaskGetTickCount();

			ESP_LOGW("PERF", "Core0 = %g%% Core1 = %g%%", (double)core0idle_count / 200.0f * 100.0f, (double)core1idle_count / 200.0f * 100.0f);
			core0idle_count = 0;
			core1idle_count = 0;

            vTaskDelayUntil(&xLastWakeTime, 1000 / portTICK_PERIOD_MS);
            // vTaskDelay(params.flush_delay / portTICK_PERIOD_MS);
        }
        vTaskDelete(NULL);
}

// program settings

// #define RECEIVER

#define ENABLE_COUNTER
#define ENABLE_DHT
#define ENABLE_TMP
#define ENABLE_MS
#define ENABLE_BMP
#define ENABLE_PMS
#define ENABLE_GPS
#define ENABLE_SD
#define ENABLE_SERVO
// #define TEST_SERVO

// #define WAIT_FOR_DEBUG

#ifdef RECEIVER
	const char *hostname = "xtrstation"; // receiver
	#include "GroundStationCode.h"
#else
	const char *hostname = "xtrcansat"; // transmitter
	#include "CansatSpecificCode.h"
#endif


namespace Startup {

	void startOTA() {
		xTaskCreate([](void*){
			// vTaskDelay(5000 / portTICK_PERIOD_MS);
			OTAService::testBegin(hostname);

			// debugD("%d free space OTA", uxTaskGetStackHighWaterMark(NULL));

			vTaskDelete(NULL);
		}, "startOTA", 16*1024, NULL, 3, NULL);
	}

	void startSafeMode() {
		ESP_LOGE("app_main", "Entering safe mode!\n");
		OTAService::beginBasic("cansatsafemode");
		vTaskDelete(NULL);
		while(true);
	}

	void startDHT() {
		// asynchronously start a sensor
		dht22.addQueue(&sendQueue);
		dht22.addQueue(&saveQueue);
		dht22.Sensor::begin(700, 4, [](){
			dht22.start(GPIO_NUM_4);
		});
	}

	void startTMP() {
		tmp102.addQueue(&saveQueue);
		tmp102.Sensor::begin(1000, 6, [](){
			tmp102.start();
		});
	}

	void startMS() {
		ms5611.addQueue(&sendQueue);
		ms5611.addQueue(&saveQueue);
		ms5611.Sensor::begin(400, 6, [](){
			ms5611.start();
		});
	}

	void startBMP() {
		bmp180.addQueue(&sendQueue);
		bmp180.addQueue(&saveQueue);
		bmp180.Sensor::begin(400, 6, [](){
			bmp180.start();
		});
	}

	void startPMS() {
		pms5003.addQueue(&saveQueue);
		pms5003.Sensor::begin(2000, 3, [](){
			pms5003.start(GPIO_NUM_17, GPIO_NUM_16);
		});
	}

	void startGPS() {
		gps.addQueue(&sendQueue);
		gps.addQueue(&saveQueue);
		gps.Sensor::begin(3000, 3, [](){
			gps.start();
		});
	}

	void startSD() {
		xTaskCreate([](void*){
			DataStorage::begin();

			vTaskDelete(NULL);
		}, "startSD", 4*1024, NULL, 3, NULL);
	}

} // namespace Startup

extern "C" void app_main() {
	ESP_LOGI("app_main", "System starting...\n");

	// initialize the arduino component
	initArduino();
	pinMode(SAFE_MODE_PIN, INPUT_PULLUP);
	Serial.begin(115200);

	// check the safe mode pin and enter safe mode if needed
	if (digitalRead(SAFE_MODE_PIN) == LOW) {
		Startup::startSafeMode();
		// at this point the program is in safe mode
		return;
	}

	// xTaskCreatePinnedToCore(core0Idle, "idle0", 1024, NULL, 0, NULL, 0);
	// xTaskCreatePinnedToCore(core1Idle, "idle1", 1024, NULL, 0, NULL, 0);
	// xTaskCreate(clearCount, "clearCount", 4096, NULL, 5, NULL);

	// create mutexes
	lora_mutex = xSemaphoreCreateMutex();
	i2c_mutex = xSemaphoreCreateMutex();

	sendQueue.begin(256);
	saveQueue.begin(256);

	// start the radio
	radio.begin(12, -1, 22, 4346E5); // 434.6 MHz

#ifdef RECEIVER // is receiver
	// startOTA();

	// set up the ground station receiver if we are the receiver
	radio->startReceive(GroundStation::onReceive, PACKET_SIZE);

	GroundStation::beginSerial();
#else // is transmitter

	// start i2c if we are the probe
	Wire.begin(14, 26/*, 400000*/);

	#ifdef ENABLE_WIFI
		// start wifi and wait for telnet debugger connection
		Startup::startOTA();

		#ifdef WAIT_FOR_DEBUG
			while (!Debug.isConnected())
				vTaskDelay(100 / portTICK_PERIOD_MS);

			vTaskDelay(1000 / portTICK_PERIOD_MS);
			debugW("System starting...");
			vTaskDelay(2000 / portTICK_PERIOD_MS);
		#endif
	#endif

	#ifdef ENABLE_SD
		Startup::startSD();
	#endif

	#ifdef ENABLE_COUNTER
		xTaskCreate(loopTask, "loopTask", 3*1024, NULL, 2, NULL);
		vTaskDelay(10 / portTICK_PERIOD_MS);
	#endif

	#ifdef ENABLE_MS
		Startup::startMS();
		// bigger delay to make MS and TMP read at different times
		vTaskDelay(50 / portTICK_PERIOD_MS);
	#endif

	#ifdef ENABLE_TMP
		Startup::startTMP();
		vTaskDelay(10 / portTICK_PERIOD_MS);
	#endif

	#ifdef ENABLE_BMP
		Startup::startBMP();
		vTaskDelay(50 / portTICK_PERIOD_MS);
	#endif

	#ifdef ENABLE_DHT
		Startup::startDHT();
		vTaskDelay(10 / portTICK_PERIOD_MS);
	#endif

	#ifdef ENABLE_PMS
		Startup::startPMS();
		vTaskDelay(10 / portTICK_PERIOD_MS);
	#endif

	#ifdef ENABLE_GPS
		Startup::startGPS();
		// small delay to offset sensor readings
		vTaskDelay(10 / portTICK_PERIOD_MS);
	#endif

	#ifdef ENABLE_SERVO
		// task will open the valve at the target altitude
		xTaskCreate(Cansat::valveTask, "valveTask", 3*1024, NULL, 5, NULL);
		vTaskDelay(10 / portTICK_PERIOD_MS);
	#endif

	#ifdef TEST_SERVO
		// task for testing the valve
		xTaskCreate(Cansat::testServoTask, "servoTest", 4*1024, NULL, 3, NULL);
		vTaskDelay(10 / portTICK_PERIOD_MS);
	#endif

	// start flushing the queue
	sendQueue.setFlushFunction(queueDataParser, 1400, "sendQueue", 4);
	vTaskDelay(50 / portTICK_PERIOD_MS);
	saveQueue.setFlushFunction(saveDataParser, 1000, "saveQueue", 3);

	// start receiving data from ground station
	// radio.startReceive(Cansat::onReceive);

#endif // RECEIVER

    ESP_LOGI("app_main", "Setup done\n");
    debugI("Setup done");
}
