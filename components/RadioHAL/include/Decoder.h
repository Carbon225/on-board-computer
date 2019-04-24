#ifndef _DECODER_H_
#define _DECODER_H_

#include <cstdio>
#include <cstring>

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "DataQueue.h"

#define PACKET_SIZE 50

struct LocationDataEncode {
    int32_t lat;
    int32_t lng;
    uint32_t alt;
};

// decode packet
int parseData(uint8_t data[PACKET_SIZE], void (*parser)(DataQueue::QueueElement))
{
    // check packet start
    if (data[0] != 0x00)
        return 1;

    // verify packet 
    uint8_t sum = 0;
    for (int i = 0; i < PACKET_SIZE - 1; i++)
    {
        sum += data[i];
    }

    if (data[PACKET_SIZE - 1] != sum)
        return 1;

    // read time
    uint16_t time = 0;
    memcpy((void*)&time, data + 1, sizeof(DataQueue::QueueElement::time));

    // go through every byte
    for (int i = 1 + sizeof(DataQueue::QueueElement::time); i < PACKET_SIZE - 1;
    {
        DataQueue::QueueElement element;
        DataQueue::DataUnion dataUnion;

        // read data type
        uint8_t dataType = data[i];

        switch (dataType)
        {
        case DataTypes::Counter:
            // for every type copy x bytes from packet into queue element
            memcpy((void *)&(dataUnion.intValue), data + i + 1, sizeof(int));
            element.type = DataTypes::Counter;
            // skip x bytes
            i += sizeof(int) + 1;
            break;

        case DataTypes::Temperature:
            memcpy((void *)&(dataUnion.floatValue), data + i + 1, sizeof(float));
            element.type = DataTypes::Temperature;
            i += sizeof(float) + 1;
            break;

        case DataTypes::TemperatureMS:
            memcpy((void *)&(dataUnion.doubleValue), data + i + 1, sizeof(double));
            element.type = DataTypes::TemperatureMS;
            i += sizeof(double) + 1;
            break;

        case DataTypes::TemperatureTMP:
            memcpy((void *)&(dataUnion.floatValue), data + i + 1, sizeof(float));
            element.type = DataTypes::TemperatureTMP;
            i += sizeof(float) + 1;
            break;

        case DataTypes::Pressure:
            memcpy((void *)&(dataUnion.longValue), data + i + 1, sizeof(long));
            element.type = DataTypes::Pressure;
            i += sizeof(long) + 1;
            break;

        case DataTypes::Humidity:
            memcpy((void *)&(dataUnion.floatValue), data + i + 1, sizeof(float));
            element.type = DataTypes::Humidity;
            i += sizeof(float) + 1;
            break;

        case DataTypes::PMS5003:
            memcpy((void *)&(dataUnion.pmsValue), data + i + 1, sizeof(PMSData));
            element.type = DataTypes::PMS5003;
            i += sizeof(PMSData) + 1;
            break;

        case DataTypes::MPU6050:
            memcpy((void *)&(dataUnion.mpuData), data + i + 1, sizeof(MPUData));
            element.type = DataTypes::MPU6050;
            i += sizeof(MPUData) + 1;
            break;

        case DataTypes::LocationData:
            // read int as double
            {
                LocationDataEncode locationEncode;
                // copy data into empty struct
                memcpy((void *)&(locationEncode), data + i + 1, sizeof(LocationDataEncode));
                // convert back to double
                dataUnion.locationData.lat = (double)locationEncode.lat / 1E4;
                dataUnion.locationData.lng = (double)locationEncode.lng / 1E4;
                dataUnion.locationData.alt = locationEncode.alt;
            }
			element.type = DataTypes::LocationData;
			i += sizeof(LocationDataEncode) + 1;
			break;

        case DataTypes::ErrorInfo:
			memcpy((void *)&(dataUnion.errorInfo), data + i + 1, sizeof(ErrorTypes::ErrorType));
			element.type = DataTypes::ErrorInfo;
			i += sizeof(ErrorTypes::ErrorType) + 1;
			break;

        case DataTypes::TemperatureBMP:
            memcpy((void *)&(dataUnion.floatValue), data + i + 1, sizeof(float));
            element.type = DataTypes::TemperatureBMP;
            i += sizeof(float) + 1;
            break;

        case DataTypes::PressureBMP:
            memcpy((void *)&(dataUnion.longValue), data + i + 1, sizeof(long));
            element.type = DataTypes::PressureBMP;
            i += sizeof(long) + 1;
            break;

        case 0xff: // packet end
            i = PACKET_SIZE;
            continue;

        default:
            return 2;
            continue;
        }

        element.time = time;
        element.data = dataUnion;
        parser(element);
    }

    return 0;
}

// encode element to packet
int encode(DataQueue::QueueElement element, uint8_t *target) {
    *target = (uint8_t) element.type;

    switch (element.type)
    {
    case DataTypes::Counter:
        // for every type copy queue element into packet
        memcpy((void*) target + 1, (void*)&(element.data.intValue), sizeof(int));
        // return data size
        return sizeof(int) + 1;
        break;

    case DataTypes::Temperature:
        memcpy((void*) target + 1, (void *)&(element.data.floatValue), sizeof(float));
        return sizeof(float) + 1;
        break;

    case DataTypes::TemperatureMS:
        memcpy((void*) target + 1, (void *)&(element.data.doubleValue), sizeof(double));
        return sizeof(double) + 1;
        break;

    case DataTypes::TemperatureTMP:
        memcpy((void*) target + 1, (void *)&(element.data.floatValue), sizeof(float));
        return sizeof(float) + 1;
        break;

    case DataTypes::Pressure:
        memcpy((void*) target + 1, (void *)&(element.data.longValue), sizeof(long));
        return sizeof(long) + 1;
        break;

    case DataTypes::Humidity:
        memcpy((void*) target + 1, (void *)&(element.data.floatValue), sizeof(float));
        return sizeof(float) + 1;
        break;

    case DataTypes::PMS5003:
        memcpy((void*) target + 1, (void *)&(element.data.pmsValue), sizeof(PMSData));
        return sizeof(PMSData) + 1;
        break;

    case DataTypes::MPU6050:
        memcpy((void*) target + 1, (void *)&(element.data.mpuData), sizeof(MPUData));
        return sizeof(MPUData) + 1;
        break;

    case DataTypes::LocationData:
        // save double as int32
        {
            // create new struct
            LocationDataEncode locationEncode = {
                (int32_t)(element.data.locationData.lat * 1E4),
                (int32_t)(element.data.locationData.lng * 1E4),
                element.data.locationData.alt
            };
            // copy struct into packet
		    memcpy((void*) target + 1, (void *)&(locationEncode), sizeof(LocationDataEncode));
        }
		return sizeof(LocationDataEncode) + 1;
		break;

    case DataTypes::ErrorInfo:
		memcpy((void*) target + 1, (void *)&(element.data.errorInfo), sizeof(ErrorTypes::ErrorType));
		return sizeof(ErrorTypes::ErrorType) + 1;
		break;

    case DataTypes::TemperatureBMP:
        memcpy((void*) target + 1, (void *)&(element.data.floatValue), sizeof(float));
        return sizeof(float) + 1;
        break;

    case DataTypes::PressureBMP:
        memcpy((void*) target + 1, (void *)&(element.data.longValue), sizeof(long));
        return sizeof(long) + 1;
        break;

    default:
        return 0;
    }
}

void generateSampleData(uint8_t target[PACKET_SIZE]) {
    uint8_t sampleData[PACKET_SIZE] = {
        (uint8_t)0x00, // packet start

        (uint8_t)0x01, // data type is temp
        (uint8_t)0x00, // value (float to hex)
        (uint8_t)0x00,
        (uint8_t)0xb0,
        (uint8_t)0x41,

        (uint8_t)0x01, // temp
        (uint8_t)0x9a,
        (uint8_t)0x99,
        (uint8_t)0xc5,
        (uint8_t)0x41,

        (uint8_t)0x02, // data type humidity
        (uint8_t)0x35,
        (uint8_t)0x00,
        (uint8_t)0x00,
        (uint8_t)0x00,

        (uint8_t)0x01, // temp
        (uint8_t)0xcd,
        (uint8_t)0xcc,
        (uint8_t)0xa8,
        (uint8_t)0x41,

        (uint8_t)0x00, // counter
        (uint8_t)0x31,
        (uint8_t)0x00,
        (uint8_t)0x00,
        (uint8_t)0x00,

        (uint8_t)0xff, // packet end
        (uint8_t)0xff, // rest 0xff
    };

    // fill with 0xff
    for (int i = 33; i < PACKET_SIZE; i++)
    {
        sampleData[i] = 0xff;
    }

    // generate packet verify
    uint8_t sum = 0;
    for (int i = 0; i < PACKET_SIZE - 1; i++)
    {
        sum += sampleData[i];
    }
    sampleData[PACKET_SIZE - 1] = sum;

    memcpy(target, sampleData, PACKET_SIZE);
}

int testDecoder(int argc, char const *argv[])
{
    uint8_t sampleData[PACKET_SIZE];
    generateSampleData(sampleData);

    parseData(sampleData, [](DataQueue::QueueElement element) {
        switch (element.type)
        {
        case DataTypes::Temperature:
            printf("Temperature = %f\n", element.data.floatValue);
            break;

        case DataTypes::Humidity:
            printf("Humidity = %d\n", element.data.intValue);
            break;

        default:
            printf("Unknown data type\n");
            break;
        }
    });

    return 0;
}

#endif
