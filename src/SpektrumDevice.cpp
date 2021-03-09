#include "SpektrumDevice.h"

#include "esp_log.h"

#ifndef min
#define min(a, b) ((a < b) ? a : b)
#endif

static const char* TAG = "Spektrum";

bool SpektrumDevice::configure(i2c_port_t i2c_port, uint8_t sda, uint8_t scl, uint8_t addr) {
  if (initialized) {
    ESP_LOGW(TAG, "SpektrumDevice::configure() called after init()!");
    return false;
  }

  _i2c_port = i2c_port;
  _sda = sda;
  _scl = scl;
  _addr = addr;

  return true;
}

bool SpektrumDevice::init() {
#if CONFIG_SPM_DEBUG
  ESP_LOGD(TAG, "SpektrumDevice::init()");
#endif
  if (initialized) {
    ESP_LOGW(TAG, "SpektrumDevice::init() called after already initialized!");
    return true;
  }

  _data[0].raw[0] = _addr;
  _data[1].raw[0] = _addr;

  initialized = i2c_slave_driver_initialize() == ESP_OK;
  if (initialized) {
    write();
  }
  return initialized;
}

esp_err_t SpektrumDevice::i2c_slave_driver_initialize() {
#if CONFIG_SPM_DEBUG
  ESP_LOGD(TAG, "Initializing i2c slave, SDA: %d, SCL: %d, i2c_port: %d, addr: %d", _sda, _scl, (int)_i2c_port, _addr);
#endif
  auto *conf = new(i2c_config_t);
  conf->mode = I2C_MODE_SLAVE;
  conf->sda_io_num = gpio_num_t(_sda);
  conf->sda_pullup_en = GPIO_PULLUP_ENABLE;
  conf->scl_io_num = gpio_num_t(_scl);
  conf->scl_pullup_en = GPIO_PULLUP_ENABLE;
  conf->slave = {
      .addr_10bit_en = 0,
      .slave_addr = (uint16_t)(_addr & 0x7F)
  };
  esp_err_t err = i2c_param_config(_i2c_port, conf);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "i2c_param_config failed: %d", err);
    return err;
  }
  err = i2c_driver_install(_i2c_port, I2C_MODE_SLAVE, CONFIG_SPM_RX_BUF_LEN, CONFIG_SPM_TX_BUF_LEN, 0);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "i2c_driver_install failed: %d", err);
  }
  return err;
}

UN_TELEMETRY *SpektrumDevice::retrieveAndSwapData() {
  lock();
  UN_TELEMETRY *ptr = data; // get pointer to existing data
  data = &_data[data == &_data[0] ? 1 : 0]; // swap buffers
  unlock();
  return ptr;
}

bool SpektrumDevice::unlock() {
  bool success = xSemaphoreGive(_mutex) == pdTRUE;
  if (!success) {
    ESP_LOGW(TAG, "SpektrumDevice::unlock() failed");
  }
  return success;
}

bool SpektrumDevice::lock(TickType_t xTicksToWait) {
  bool success = xSemaphoreTake(_mutex, xTicksToWait) == pdTRUE;
  if (!success) {
    ESP_LOGW(TAG, "SpektrumDevice::lock() failed");
  }
  return success;
}

int SpektrumDevice::write() {
  UN_TELEMETRY* ptr = retrieveAndSwapData();
#if CONFIG_SPM_DEBUG
  TickType_t ticksBegin = xTaskGetTickCount();
#endif
  int size = i2c_slave_write_buffer(_i2c_port, (uint8_t*)ptr->raw, _packetSize, CONFIG_SPM_WRITE_WAIT_MS / portTICK_RATE_MS);
  #if CONFIG_SPM_DEBUG
    ESP_LOGD(TAG, "Wrote %d bytes from i2c slave after %d millis", size, (int)((xTaskGetTickCount() - ticksBegin) / portTICK_RATE_MS));
  #endif
  return size;
}

static void __attribute__ ((noreturn)) _task(void *pvParameter) {
  auto *device = reinterpret_cast<SpektrumDevice*>(pvParameter);
  while (true) {
    // todo: currently due to either issues or limitation of i2c slave implementation on ESP32/ESP-IDF,
    //       this code will continuously try to write to the i2c master, without waiting
    //       for the master to be ready. If it is not ready, this code will block until then.
    //       However, because this is on a separate task, the blocking is async to the rest of
    //       the codebase.
    taskYIELD();
    device->write();
  }
}

bool SpektrumDevice::start(portBASE_TYPE priority, uint32_t stackSize, BaseType_t coreId) {
#if CONFIG_SPM_DEBUG
  ESP_LOGD(TAG, "Creating Spektrum Device FreeRTOS Task, stack: %d, priority: %d, core: %d", stackSize, priority, coreId);
#endif
  return xTaskCreatePinnedToCore(&_task, "Spektrum Device", stackSize, this, priority, &_handle, coreId) == pdPASS;
}
