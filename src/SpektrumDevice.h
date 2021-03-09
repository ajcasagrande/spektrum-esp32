#pragma once

#include "sdkconfig.h"

#include <cstdint>
#include <esp_err.h>
#include <esp_event.h>
#include <driver/i2c.h>

typedef  uint8_t UINT8;
typedef uint16_t UINT16;
typedef uint32_t UINT32;
typedef uint64_t UINT64;

typedef  int8_t INT8;
typedef int16_t INT16;
typedef int32_t INT32;
typedef int64_t INT64;

#include "SpektrumDocumentation/Telemetry/spektrumTelemetrySensors.h"

#define SPM_LOCK(device, code) { if ((device).lock()) { code; (device).unlock(); } }

class SpektrumDevice {
private:
  bool initialized = false;
  TaskHandle_t _handle;
  uint8_t _sda = CONFIG_SPM_I2C_SDA;
  uint8_t _scl = CONFIG_SPM_I2C_SCL;
  uint8_t _addr = CONFIG_SPM_I2C_ADDRESS;
  i2c_port_t _i2c_port = (i2c_port_t) CONFIG_SPM_I2C_NUM;
  size_t _packetSize = sizeof(UN_TELEMETRY);

  UN_TELEMETRY _data[2] = {UN_TELEMETRY(), UN_TELEMETRY()};

  QueueHandle_t _mutex = xSemaphoreCreateMutex();
  UN_TELEMETRY* retrieveAndSwapData();
  esp_err_t i2c_slave_driver_initialize();

public:
  UN_TELEMETRY *data = &_data[0];

  bool init();
  bool lock(TickType_t xTicksToWait=CONFIG_SPM_LOCK_WAIT_MILLIS/portTICK_RATE_MS);
  bool unlock();
  bool start(portBASE_TYPE priority = 5, uint32_t stackSize = 2048, BaseType_t coreId = 0);
  int write();

  bool configure(i2c_port_t i2c_port=(i2c_port_t)CONFIG_SPM_I2C_NUM, uint8_t sda=CONFIG_SPM_I2C_SDA, uint8_t scl=CONFIG_SPM_I2C_SCL, uint8_t addr=CONFIG_SPM_I2C_ADDRESS);
};
