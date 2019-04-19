#ifndef _MPUHAL_H_
#define _MPUHAL_H_

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#include "MPU.hpp"
#include "mpu/math.hpp"
#include "mpu/types.hpp"

#include "I2Cbus.hpp"

#include "DataQueue.h"
#include "Sensor.h"

#include "RemoteDebug.h"
#include "Wire.h"

extern RemoteDebug Debug;


namespace MPU_COMMON {
	const char *m_TAG = "mpu6050";

	static  I2C_t& i2c                     = i2c0;
	static constexpr gpio_num_t SDA       = GPIO_NUM_14;
	static constexpr gpio_num_t SCL       = GPIO_NUM_26;
	static constexpr uint32_t CLOCK_SPEED = 400000;  // 400 KHz

	static constexpr int kInterruptPin         = 27;  // GPIO_NUM
	static constexpr uint16_t kSampleRate      = 250;  // Hz
	static constexpr mpud::accel_fs_t kAccelFS = mpud::ACCEL_FS_4G;
	static constexpr mpud::gyro_fs_t kGyroFS   = mpud::GYRO_FS_500DPS;
	static constexpr mpud::dlpf_t kDLPF        = mpud::DLPF_98HZ;
	static constexpr mpud::int_config_t kInterruptConfig = {
	    .level = mpud::INT_LVL_ACTIVE_HIGH,
	    .drive = mpud::INT_DRV_PUSHPULL,
	    .mode  = mpud::INT_MODE_PULSE50US,
	    .clear = mpud::INT_CLEAR_STATUS_REG
	};
};

static IRAM_ATTR void mpuISR(TaskHandle_t taskHandle) {
	BaseType_t HPTaskWoken = pdFALSE;
	vTaskNotifyGiveFromISR(taskHandle, &HPTaskWoken);
	if (HPTaskWoken == pdTRUE) portYIELD_FROM_ISR();
}

using namespace MPU_COMMON;

class MPUHAL : public Sensor, private MPU_t {
private:
	float roll{0}, pitch{0}, yaw{0};
	mpud::float_axes_t accelG;

	static void mpuTask(void *mpu_void) {
		MPUHAL *mpu = (MPUHAL*) mpu_void;
		mpu->setBus(i2c);
		mpu->setAddr(mpud::MPU_I2CADDRESS_AD0_LOW);

		// Verify connection
		while (esp_err_t err = mpu->testConnection()) {
			ESP_LOGE(m_TAG, "Failed to connect to the MPU, error=%#X", err);
			vTaskDelay(1000 / portTICK_PERIOD_MS);
		}
		ESP_LOGI(m_TAG, "MPU connection successful!");

		// Initialize
		ESP_ERROR_CHECK(mpu->initialize());

		// Self-Test
		mpud::selftest_t retSelfTest;
		while (esp_err_t err = mpu->selfTest(&retSelfTest)) {
			ESP_LOGE(m_TAG, "Failed to perform MPU Self-Test, error=%#X", err);
			vTaskDelay(1000 / portTICK_PERIOD_MS);
		}
		ESP_LOGI(m_TAG, "MPU Self-Test result: Gyro=%s Accel=%s",  //
				 (retSelfTest & mpud::SELF_TEST_GYRO_FAIL ? "FAIL" : "OK"),
				 (retSelfTest & mpud::SELF_TEST_ACCEL_FAIL ? "FAIL" : "OK"));

		// Calibrate
		mpud::raw_axes_t accelBias, gyroBias;
		ESP_ERROR_CHECK(mpu->computeOffsets(&accelBias, &gyroBias));
		ESP_ERROR_CHECK(mpu->setAccelOffset(accelBias));
		ESP_ERROR_CHECK(mpu->setGyroOffset(gyroBias));

		// Configure
		ESP_ERROR_CHECK(mpu->setAccelFullScale(kAccelFS));
		ESP_ERROR_CHECK(mpu->setGyroFullScale(kGyroFS));
		ESP_ERROR_CHECK(mpu->setSampleRate(kSampleRate));
		ESP_ERROR_CHECK(mpu->setDigitalLowPassFilter(kDLPF));

		// Setup FIFO
		ESP_ERROR_CHECK(mpu->setFIFOConfig(mpud::FIFO_CFG_ACCEL | mpud::FIFO_CFG_GYRO));
		ESP_ERROR_CHECK(mpu->setFIFOEnabled(true));
		constexpr uint16_t kFIFOPacketSize = 12;

		// Setup Interrupt
		constexpr gpio_config_t kGPIOConfig = {
			.pin_bit_mask = (uint64_t) 0x1 << kInterruptPin,
			.mode         = GPIO_MODE_INPUT,
			.pull_up_en   = GPIO_PULLUP_DISABLE,
			.pull_down_en = GPIO_PULLDOWN_ENABLE,
			.intr_type    = GPIO_INTR_POSEDGE  //
		};
		gpio_config(&kGPIOConfig);
		gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
		gpio_isr_handler_add((gpio_num_t) kInterruptPin, mpuISR, xTaskGetCurrentTaskHandle());
		ESP_ERROR_CHECK(mpu->setInterruptConfig(kInterruptConfig));
		ESP_ERROR_CHECK(mpu->setInterruptEnabled(mpud::INT_EN_RAWDATA_READY));

		// Ready to start reading
		ESP_ERROR_CHECK(mpu->resetFIFO());  // start clean

		// Reading Loop
		while (true) {
			// Wait for notification from mpuISR
			uint32_t notificationValue = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
			if (notificationValue > 1) {
				ESP_LOGW(m_TAG, "Task Notification higher than 1, value: %d", notificationValue);
				mpu->resetFIFO();
				continue;
			}
			// Check FIFO count
			uint16_t fifocount = mpu->getFIFOCount();
			if (esp_err_t err = mpu->lastError()) {
				ESP_LOGE(m_TAG, "Error reading fifo count, %#X", err);
				mpu->resetFIFO();
				continue;
			}
			if (fifocount > kFIFOPacketSize * 2) {
				if (!(fifocount % kFIFOPacketSize)) {
					ESP_LOGE(m_TAG, "Sample Rate too high!, not keeping up the pace!, count: %d", fifocount);
				}
				else {
					ESP_LOGE(m_TAG, "FIFO Count misaligned! Expected: %d, Actual: %d", kFIFOPacketSize, fifocount);
				}
				mpu->resetFIFO();
				continue;
			}
			// Burst read data from FIFO
			uint8_t buffer[kFIFOPacketSize];
			if (esp_err_t err = mpu->readFIFO(kFIFOPacketSize, buffer)) {
				ESP_LOGE(m_TAG, "Error reading sensor data, %#X", err);
				mpu->resetFIFO();
				continue;
			}
			// Format
			mpud::raw_axes_t rawAccel, rawGyro;
			rawAccel.x = buffer[0] << 8 | buffer[1];
			rawAccel.y = buffer[2] << 8 | buffer[3];
			rawAccel.z = buffer[4] << 8 | buffer[5];
			rawGyro.x  = buffer[6] << 8 | buffer[7];
			rawGyro.y  = buffer[8] << 8 | buffer[9];
			rawGyro.z  = buffer[10] << 8 | buffer[11];
			// Calculate tilt angle
			// range: (roll[-180,180]  pitch[-90,90]  yaw[-180,180])
			constexpr double kRadToDeg = 57.2957795131;
			constexpr float kDeltaTime = 1.f / kSampleRate;
			float gyroRoll             = mpu->roll + mpud::math::gyroDegPerSec(rawGyro.x, kGyroFS) * kDeltaTime;
			float gyroPitch            = mpu->pitch + mpud::math::gyroDegPerSec(rawGyro.y, kGyroFS) * kDeltaTime;
			float gyroYaw              = mpu->yaw + mpud::math::gyroDegPerSec(rawGyro.z, kGyroFS) * kDeltaTime;
			float accelRoll            = atan2(-rawAccel.x, rawAccel.z) * kRadToDeg;
			float accelPitch = atan2(rawAccel.y, sqrt(rawAccel.x * rawAccel.x + rawAccel.z * rawAccel.z)) * kRadToDeg;
			// Fusion
			mpu->roll  = gyroRoll * 0.95f + accelRoll * 0.05f;
			mpu->pitch = gyroPitch * 0.95f + accelPitch * 0.05f;
			mpu->yaw   = gyroYaw;
			// correct yaw
			if (mpu->yaw > 180.f)
				mpu->yaw -= 360.f;
			else if (mpu->yaw < -180.f)
				mpu->yaw += 360.f;

			mpu->accelG = mpud::accelGravity(rawAccel, mpud::ACCEL_FS_4G);
		}
		vTaskDelete(nullptr);
	}

protected:
	static int n_mpus;

    virtual DataQueue::QueueElement read() {
    	DataQueue::DataUnion data;
    	data.mpuData = {
    			roll,
				pitch,
				yaw,

				accelG.x,
				accelG.y,
				accelG.z
    	};
    	DataQueue::QueueElement element = {
    			.type = DataTypes::MPU6050,
				.data = data,
				.time = (uint16_t) (millis() / 1000)
    	};

        return element;
    }

public:
    MPUHAL(const char *const pcName)
        : Sensor(pcName), MPU_t()
    {
    	if (n_mpus > 0) {
    		ESP_LOGE(m_TAG, "Maximum number of MPUs reached");
    		return;
    	}
        ESP_LOGI(m_TAG, "Starting MPU...\n");

        i2c.begin(GPIO_NUM_14, GPIO_NUM_26, CLOCK_SPEED);
        xTaskCreate(mpuTask, pcName, 4 * 1024, this, 6, NULL);
        n_mpus++;
    }

    virtual ~MPUHAL() {

    }
};

int MPUHAL::n_mpus = 0;

#endif
