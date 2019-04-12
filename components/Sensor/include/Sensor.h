#ifndef _SENSOR_H_
#define _SENSOR_H_

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "DataQueue.h"

class Sensor {
private:
	TaskHandle_t _readTaskHandle = NULL;

    // universal task for reading sensor
    // has to be static because otherwise it couldn't be passed as function pointer
    static void genericSensorReadTask(void *pvParameters) {
        // unpack pointer to current sensor object
        Sensor *sensor = (Sensor*) pvParameters;

        TickType_t xLastWakeTime;
        for (;;) {
            xLastWakeTime = xTaskGetTickCount();

            // send element from virtual read() to all queues
            sensor->sendToQueues(sensor->read());

            // sleep
            vTaskDelayUntil(&xLastWakeTime, sensor->m_read_delay / portTICK_PERIOD_MS);
        }
        sensor->_readTaskHandle = NULL;
        vTaskDelete(NULL);
    }

    // name of read task
    const char *const m_pcName;

    // array of used queues (where we store data)
    DataQueue::Queue **m_queues;
    int m_nQueues = 0;
    const int m_maxQueues;

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
    Sensor(const char *const pcName, int n_queues)
        : m_pcName(pcName),
          m_maxQueues(n_queues),
		  m_read_delay(1000)
    {
        // create array of DataQueue::Queue pointers
        m_queues = new DataQueue::Queue *[n_queues];
    }

    virtual ~Sensor() {
    	// stop reading sensor
    	if (_readTaskHandle != NULL)
    		vTaskDelete(_readTaskHandle);

        // important to avoid memory leaks
        delete [] m_queues;
    }

    // call before begin() to subscribe sensor to a queue (it will use to for data storage)
    void addQueue(DataQueue::Queue *queue) {
        m_queues[m_nQueues] = queue;
        m_nQueues++;
    }

    // start reading the sensor and storing data
    void begin(unsigned long read_delay, int read_priority) {
        m_read_delay = read_delay;

        // pass this pointer to the static function
        xTaskCreate(genericSensorReadTask, m_pcName, 2048, this, read_priority, &_readTaskHandle);

        // TaskManager::scheduleTask(genericSensorReadTask, m_pcName, 0, this, read_priority);
    }

    void flush() {
        // flush all data from m_queues
    }
};

#endif
