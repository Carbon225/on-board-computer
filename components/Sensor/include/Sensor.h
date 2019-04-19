#ifndef _SENSOR_H_
#define _SENSOR_H_

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "DataQueue.h"

typedef void (*beginSensor_fn)();

class Sensor {
private:
	TaskHandle_t _readTaskHandle = NULL;

    // universal task for reading sensor
    // has to be static because otherwise it couldn't be passed as function pointer to xTaskCreate
    static void genericSensorReadTask(void *pvParameters) {
        // unpack pointer to current sensor object
        Sensor *sensor = (Sensor*) pvParameters;

        sensor->m_beginSensor();

        TickType_t xLastWakeTime;
        for (;;) {
            xLastWakeTime = xTaskGetTickCount();

            // send element from virtual read() to all queues
            sensor->sendToQueues(sensor->read());

            debugD("%d free space", uxTaskGetStackHighWaterMark(NULL));

            // sleep
            // vTaskDelayUntil(&xLastWakeTime, sensor->m_read_delay / portTICK_PERIOD_MS);
            vTaskDelay(sensor->m_read_delay / portTICK_PERIOD_MS);
        }
        sensor->_readTaskHandle = NULL;
        vTaskDelete(NULL);
    }

    // name of read task
    const char *const m_pcName;

    // function to start the hardware sensor
    beginSensor_fn m_beginSensor = NULL;

    static const int m_maxQueues = 2;

    // array of used queues (where we store data)
    DataQueue::Queue *m_queues[m_maxQueues];
    int m_nQueues = 0;

    // how often to read
    int m_read_delay;

protected:
    // handles sensor reading (override in derived class)
    virtual DataQueue::QueueElement read() = 0;

    // to be called from read(), adds element to set queues
    void sendToQueues(DataQueue::QueueElement element) {
        for (int i = 0; i < m_nQueues; i++) {
            m_queues[i]->add(&element);
        }
    }

public:
    Sensor(const char *const pcName)
        : m_pcName(pcName),
		  m_read_delay(1000)
    {
        // create array of DataQueue::Queue pointers
        // m_queues = new DataQueue::Queue *[n_queues];
    }

    virtual ~Sensor() {
    	// stop reading sensor
    	if (_readTaskHandle != NULL)
    		vTaskDelete(_readTaskHandle);

        // important to avoid memory leaks
        // delete [] m_queues;
    }
    
    void stop() {
        // stop reading sensor
    	if (_readTaskHandle != NULL)
    		vTaskDelete(_readTaskHandle);
    }

    // call before begin() to subscribe sensor to a queue (it will use to for data storage)
    void addQueue(DataQueue::Queue *queue) {
        if (m_nQueues >= m_maxQueues)
            return;

        m_queues[m_nQueues] = queue;
        m_nQueues++;
    }

    // start reading the sensor and storing data
    void begin(unsigned long read_delay, int read_priority, beginSensor_fn beginSensor) {
        m_read_delay = read_delay;
        m_beginSensor = beginSensor;

        // pass this pointer to the static function
        xTaskCreate(genericSensorReadTask, m_pcName, 8*1024, this, read_priority, &_readTaskHandle);

        // TaskManager::scheduleTask(genericSensorReadTask, m_pcName, 0, this, read_priority);
    }

    void flush() {
        // flush all data from m_queues
    }
};

#endif
