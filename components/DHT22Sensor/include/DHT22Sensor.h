#ifndef _DHT22SENSOR_H_
#define _DHT22SENSOR_H_

#include "Sensor.h"
#include "DHT.h"

class DHT22Sensor : public Sensor, private DHT {
protected:
    /* you need to write this virutal function
       so the Sensor base class can use your sensor */
    virtual DataQueue::QueueElement read() {
        // errorHandler is disabled
        /*int ret =*/ DHT::readDHT();
        // DHT::errorHandler(ret);

        float temp = DHT::getTemperature();
        float hum = DHT::getHumidity();

        // create union for storing temperature
        DataQueue::DataUnion data;
        data.floatValue = hum; // assign to correct type

        // create queue element with data and type
        DataQueue::QueueElement element = {
            .type = DataTypes::Humidity, // set type for this reading
            .data = data,                 // our value union
			.time = (uint16_t) (millis() / 1000)
        };

        // send humidity to queues
        Sensor::sendToQueues(element);

        // set temperature value
        element.type = DataTypes::Temperature;
        element.data.floatValue = temp;

        // return created element to add to queues
        return element;
    }

public:
    DHT22Sensor(const char *const pcName)
        : Sensor(pcName), DHT()
    {
        
    }

    void begin(unsigned long read_delay, int read_priority, gpio_num_t pin) {
        // printf("Starting DHT...\n");
        debugI("Starting DHT...\n");
        DHT::setDHTgpio(pin);

        Sensor::begin(read_delay, read_priority);
    }

    void stop() {
        Sensor::stop();
    }
};

#endif
