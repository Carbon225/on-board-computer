#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/gpio.h"

#define ENABLE_WIFI

#include "Arduino.h"
#include "Wire.h"
#include "DataQueue.h"

#include "DHT22Sensor.h"
#include "MPUHAL.h"
#include "CounterSensor.h"
#include "PMS5003Sensor.h"
#include "GPSSensor.h"
#include "MS5611Sensor.h"
#include "TMP102Sensor.h"

#include "RemoteDebug.h"
#include "DataStorage.h"
#include "RadioHAL.h"
#include "OTAService.h"
#include "Decoder.h"


#define SAFE_MODE_PIN 13

DHT22Sensor *dht22; // (GPIO_NUM_4, "DHTn1");
PMS5003Sensor *pms5003;
MPUHAL *mpu6050;
RadioHAL *radio;
MS5611Sensor *ms5611;
TMP102Sensor *tmp102;
GPSSensor *gps;

// mutex semaphore to make sure only one task at a time can use i2c and radio
SemaphoreHandle_t i2c_mutex = NULL;
SemaphoreHandle_t lora_mutex = NULL;

// main data queue for sending data over radio
// DataQueue::Queue sendQueue(128);
DataQueue::Queue *sendQueue;


int counter = 0;

// test task to see if device works
void loopTask(void *ignore) {
    for (;;) {
		// debugD("Ich weiss Counter");
        DataQueue::DataUnion data;
        data.intValue = counter;

        DataQueue::QueueElement element = {
            .type = DataTypes::Counter,
            .data = data,
			.time = (uint16_t) (millis() / 1000)
        };

        sendQueue->add(&element);
        counter++;

		// debugD("Counter added to queue");

		debugD("%d free space", uxTaskGetStackHighWaterMark(NULL));

        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

// used to parse queue elements and compose radio packets
void queueDataParser(QueueHandle_t queue) {
	DataQueue::QueueElement element;
	uint8_t packet[PACKET_SIZE + sizeof(DataQueue::QueueElement)] = {0}; // overhead for last element
	int packet_size = 1;

	/*bool encoded_time = false;

	memcpy(packet + packet_size, element.time, sizeof(element.time));
	packetSize += sizeof(element.time);*/

	// read next element
	while (xQueuePeek(queue, &element, 0)) {
		logElement(element);

		// try to encode and get size of encoded data
		int new_element_size = encode(element, packet + packet_size);

		// if no space left in packet stop encoding
		if (packet_size + new_element_size > PACKET_SIZE) {
			break; // element was peeked so it will remain in queue
		}
		// if encoding finished set new packet size
		packet_size += new_element_size;

		// save element to storage
		DataStorage::saveElement(element);

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

	/*for (int i = 0; i < PACKET_SIZE; i++) {
		printf("%x\n", packet[i]);
	}*/

	if (packet_size > 1)
		radio->send(packet);

	debugD("%d free space", uxTaskGetStackHighWaterMark(NULL));
}

// program settings

// #define RECEIVER

// #define ENABLE_COUNTER
#define ENABLE_DHT
#define ENABLE_TMP
// #define ENABLE_MS
// #define ENABLE_MPU
// #define ENABLE_PMS
// #define ENABLE_GPS
// #define ENABLE_SD
// #define ENABLE_SERVO
// #define TEST_SERVO

#define WAIT_FOR_DEBUG

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

			debugD("%d free space OTA", uxTaskGetStackHighWaterMark(NULL));

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
		xTaskCreate([](void*){
			// start sensor with constructor
			dht22 = new DHT22Sensor(GPIO_NUM_4, "DHTn1");
			// tell it to save to sendQueue
			dht22 -> addQueue(sendQueue);
			// start the sensor class (reading every 1500 ms, task priority 3)
			dht22 -> begin(1500, 3);

			debugD("%d free space DHT", uxTaskGetStackHighWaterMark(NULL));

			vTaskDelete(NULL);
		}, "startDHT", 3*1024, NULL, 3, NULL);
	}

	void startTMP() {
		xTaskCreate([](void*){
			tmp102 = new TMP102Sensor("TMPn1");
			tmp102 -> addQueue(sendQueue);
			tmp102 -> Sensor::begin(900, 4);

			debugD("%d free space TMP", uxTaskGetStackHighWaterMark(NULL));

			vTaskDelete(NULL);
		}, "startTMP", 3*1024, NULL, 3, NULL);
	}

	void startMS() {
		xTaskCreate([](void*){
			ms5611 = new MS5611Sensor("MSn1");
			ms5611 -> addQueue(sendQueue);
			ms5611 -> Sensor::begin(100, 5);

			debugD("%d free space MS", uxTaskGetStackHighWaterMark(NULL));

			vTaskDelete(NULL);
		}, "startMS", 3*1024, NULL, 3, NULL);
	}

	void startMPU() {
		xTaskCreate([](void*){
			mpu6050 = new MPUHAL("mpun1");
			mpu6050->addQueue(sendQueue);
			mpu6050->begin(1000, 3);

			debugD("%d free space MPU", uxTaskGetStackHighWaterMark(NULL));

			vTaskDelete(NULL);
		}, "startMPU", 3*1024, NULL, 3, NULL);
	}

	void startPMS() {
		xTaskCreate([](void*){
			pms5003 = new PMS5003Sensor(UART_NUM_2, GPIO_NUM_17, GPIO_NUM_16, "pms5003");
			// pms5003 = new PMS5003Sensor(UART_NUM_2, GPIO_NUM_16, GPIO_NUM_17, "pms5003");
			pms5003->addQueue(sendQueue);
			pms5003->begin(2000, 3);

			debugD("%d free space PMS", uxTaskGetStackHighWaterMark(NULL));

			vTaskDelete(NULL);
		}, "startPMS", 3*1024, NULL, 3, NULL);
	}

	void startGPS() {
		xTaskCreate([](void*){
			gps = new GPSSensor("GPSSensor");
			gps -> addQueue(sendQueue);
			gps -> Sensor::begin(3000, 3);

			debugD("%d free space GPS", uxTaskGetStackHighWaterMark(NULL));

			vTaskDelete(NULL);
		}, "startGPS", 3*1024, NULL, 3, NULL);
	}

	void startSD() {
		xTaskCreate([](void*){
			DataStorage::begin();

			debugD("%d free space SD", uxTaskGetStackHighWaterMark(NULL));

			vTaskDelete(NULL);
		}, "startSD", 3*1024, NULL, 3, NULL);
	}

} // namespace Startup

extern "C" void app_main() {
	ESP_LOGI("app_main", "System starting...\n");

	// initialize the arduino component
	initArduino();
	pinMode(SAFE_MODE_PIN, INPUT_PULLUP);
	// Serial.begin(115200);

	// check the safe mode pin and enter safe mode if needed
	if (digitalRead(SAFE_MODE_PIN) == LOW) {
		Startup::startSafeMode();
		// at this point the program is in safe mode
		return;
	}

	// create mutexes
	lora_mutex = xSemaphoreCreateMutex();
	i2c_mutex = xSemaphoreCreateMutex();

	sendQueue = new DataQueue::Queue(256);

	// start the radio
	radio = new RadioHAL(12, -1, 22);
	radio->begin(4346E5); // 434.6 MHz

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
		vTaskDelay(400 / portTICK_PERIOD_MS);
	#endif

	#ifdef ENABLE_TMP
		Startup::startTMP();
		vTaskDelay(10 / portTICK_PERIOD_MS);
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

	#ifdef ENABLE_MPU
		Startup::startMPU();
		vTaskDelay(10 / portTICK_PERIOD_MS);
	#endif

	#ifdef ENABLE_SERVO
		// task will open the valve at the target altitude
		xTaskCreate(Cansat::valveTask, "valveTask", 3*1024, NULL, 5, NULL);
		vTaskDelay(10 / portTICK_PERIOD_MS);
	#endif

	#ifdef TEST_SERVO
		// task for testing the valve
		xTaskCreate(Cansat::testServoTask, "servoTest", 3*1024, NULL, 3, NULL);
		vTaskDelay(10 / portTICK_PERIOD_MS);
	#endif

	// start flushing the queue
	sendQueue->setFlushFunction(queueDataParser, 1300, "sendQueue", 4);

	// start receiving data from ground station
	radio->startReceive(Cansat::onReceive);

#endif // RECEIVER

    ESP_LOGI("app_main", "Setup done\n");
    debugI("Setup done");
}
