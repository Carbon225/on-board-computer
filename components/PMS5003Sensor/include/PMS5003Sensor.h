#ifndef _PMS5003SENSOR_H_
#define _PMS5003SENSOR_H_

#include "Arduino.h"
#include "Sensor.h"
#include "DataQueue.h"

class PMS5003Sensor : public Sensor {
private:
	const int m_bufSize = 1024; // min 128
	// send to pms to enable passive mode
	const char m_passive_cmd[7] = {0x42, 0x4d, 0xe1, 0x00, 0x00, 0x01, 0x70};
	// send to pms to request data
	const char m_request_cmd[7] = {0x42, 0x4d, 0xe2, 0x00, 0x00, 0x01, 0x71};
	const uart_port_t m_uart_num;

protected:
    /* you need to write this virtual function
       so the Sensor base class can use your sensor */
    virtual DataQueue::QueueElement read() {
    	// flush RX buffer to prepare for response
    	// uart_flush(m_uart_num);
    	// request data
    	uart_write_bytes(m_uart_num, m_request_cmd, 7);
    	// read 32 bytes to array, wait for 5ms if no data
		uint8_t uartMessage[m_bufSize];
		uart_read_bytes(m_uart_num, uartMessage, 32, 5 / portTICK_PERIOD_MS);

        DataQueue::DataUnion data;
        data.pmsValue = parseData(uartMessage);

        DataQueue::QueueElement element = {
        	.type = DataTypes::PMS5003,
			.data = data,
			.time = (uint16_t) (millis() / 1000)
        };

        // return created element to add to queues
        return element;
    }

public:
    PMS5003Sensor(uart_port_t uart_num, gpio_num_t rx, gpio_num_t tx,
    			  const char *const pcName, int n_queues = 5)
	: Sensor(pcName, n_queues),
	  m_uart_num(uart_num) {
        printf("Starting PMS5003...\n");
		debugD("Starting PMS5003...\n");

        /* Configure parameters of an UART driver,
		 * communication pins and install the driver */
		uart_config_t uart_config = {
			.baud_rate = 9600,
			.data_bits = UART_DATA_8_BITS,
			.parity    = UART_PARITY_DISABLE,
			.stop_bits = UART_STOP_BITS_1,
			.flow_ctrl = UART_HW_FLOWCTRL_DISABLE
		};
		uart_param_config(m_uart_num, &uart_config);
		uart_set_pin(m_uart_num, tx, rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
		uart_driver_install(m_uart_num, m_bufSize, 0, 0, NULL, 0);

		// enable sensor
		pinMode(32, OUTPUT);
		digitalWrite(32, HIGH);

		// set sensor to passive move
		uart_write_bytes(m_uart_num, m_passive_cmd, 7);
		uart_wait_tx_done(m_uart_num, 100);

		debugI("PMS5003 started\n");
    }

    virtual ~PMS5003Sensor() {
    	uart_driver_delete(m_uart_num);
    }

	// combine 2 bytes into number
    static uint16_t bytes2Int(uint8_t *data) {
    	return (uint16_t)((data[0] << 8) | (data[1]));
    }

	// decode data from pms
    static PMSData parseData(uint8_t *data) {
    	PMSData errorStruct = {
    			0xffff, 0xffff, 0xffff,
				0xffff, 0xffff, 0xffff,
				0xffff, 0xffff, 0xffff,
				0xffff, 0xffff, 0xffff,
				0xffff
    	};

    	// check first 2 bytes (start1, start2)
    	if (data[0] != 0x42 || data[1] != 0x4d)
    		return errorStruct;

    	// check frame length == 2 * 13 + 2
    	uint16_t frame_length = bytes2Int(data+2);
    	if (frame_length != 2 * 13 + 2)
    		return errorStruct;

    	// verify checksum
    	uint16_t checksum = 0;
    	for (int i = 0; i < 4 + 2 * 13; i++)
    		checksum += data[i];

    	if (checksum != bytes2Int(data+30))
    		return errorStruct;

    	/*
    	printf("Parsing: ");
		for (int i = 0; i < 32; i++) {
			printf("%.2x ", data[i]);
		}
		printf("\n");
		*/

    	// save to struct
    	PMSData dataStruct = {
			bytes2Int(data+4),
			bytes2Int(data+6),
			bytes2Int(data+8),
			bytes2Int(data+10),

			bytes2Int(data+12),
			bytes2Int(data+14),
			bytes2Int(data+16),
			bytes2Int(data+18),

			bytes2Int(data+20),
			bytes2Int(data+22),
			bytes2Int(data+24),
			bytes2Int(data+26),

			bytes2Int(data+28)
    	};
    	// printf("dataStruct.data1 = %u\n", dataStruct.data1);

    	return dataStruct;
    }
};

#endif
