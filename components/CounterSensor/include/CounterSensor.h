#ifndef _COUNTERSENSOR_H_
#define _COUNTERSENSOR_H_

#include "Arduino.h"
#include "Sensor.h"

// an example implementation of our sensor class
class CounterSensor : public Sensor {
private:
    // gets incremented every read cycle
    int m_value;

protected:
    /* you need to write this virutal function
       so the Sensor base class can use your sensor */
    virtual DataQueue::QueueElement read() {
        // create union for storing temperature
        DataQueue::DataUnion data;
        data.intValue = m_value; // assign to correct type

        // create queue element with data and type
        DataQueue::QueueElement element = {
            .type = DataTypes::Counter,  // set type for this reading
            .data = data,                 // our value union
			.time = (uint16_t) (millis() / 1000)
        };

        m_value++;

        // return created element to add to queues
        return element;
    }

public:
    CounterSensor(const char *const pcName)
    : Sensor(pcName) {
        m_value = 0;
    }
};

#endif
