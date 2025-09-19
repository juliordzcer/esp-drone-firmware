#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/i2c.h"
#include "stm32_legacy.h"
#include "i2c_drv.h"
#include "config.h"
#define DEBUG_MODULE "I2CDRV"
#include "debug_cf.h"

#include "esp_log.h" 

// <-- CAMBIO: Usamos el TAG para el logging estándar del ESP-IDF
static const char *TAG = "I2CDRV";

// Definitions of sensors I2C bus
#define I2C_DEFAULT_SENSORS_CLOCK_SPEED   400000

// Definition of eeprom and deck I2C bus
#define I2C_DEFAULT_DECK_CLOCK_SPEED      100000

// Cost definitions of busses
static const I2cDef sensorBusDef = {
    .i2cPort        = I2C_NUM_0,
    .i2cClockSpeed  = I2C_DEFAULT_SENSORS_CLOCK_SPEED,
    .gpioSCLPin     = CONFIG_I2C0_PIN_SCL,
    .gpioSDAPin     = CONFIG_I2C0_PIN_SDA,
    .gpioPullup     = GPIO_PULLUP_DISABLE,
};

I2cDrv sensorsBus = {
    .def            = &sensorBusDef,
    .is_initialized = false, // <-- CAMBIO
};

static const I2cDef deckBusDef = {
    .i2cPort        = I2C_NUM_1,
    .i2cClockSpeed  = I2C_DEFAULT_DECK_CLOCK_SPEED,
    .gpioSCLPin     = CONFIG_I2C1_PIN_SCL,
    .gpioSDAPin     = CONFIG_I2C1_PIN_SDA,
    .gpioPullup     = GPIO_PULLUP_ENABLE,
};

I2cDrv deckBus = {
    .def            = &deckBusDef,
    .is_initialized = false, // <-- CAMBIO
};

// <-- CAMBIO: La función ahora devuelve esp_err_t para un mejor manejo de errores
esp_err_t i2cdrvInit(I2cDrv *i2c)
{
    // Si ya está inicializado, no hacer nada y reportar éxito.
    if (i2c->is_initialized) {
        return ESP_OK;
    }

    // Crear el mutex la primera vez que se inicializa
    if (i2c->isBusFreeMutex == NULL) {
        i2c->isBusFreeMutex = xSemaphoreCreateMutex();
        if (i2c->isBusFreeMutex == NULL) {
            ESP_LOGE(TAG, "Failed to create mutex for I2C port %d", i2c->def->i2cPort);
            return ESP_FAIL;
        }
    }

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = i2c->def->gpioSDAPin,
        .sda_pullup_en = i2c->def->gpioPullup,
        .scl_io_num = i2c->def->gpioSCLPin,
        .scl_pullup_en = i2c->def->gpioPullup,
        .master.clk_speed = i2c->def->i2cClockSpeed,
    };

    esp_err_t err = i2c_param_config(i2c->def->i2cPort, &conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C#%d param config failed. Error: %s", i2c->def->i2cPort, esp_err_to_name(err));
        return err;
    }

    err = i2c_driver_install(i2c->def->i2cPort, conf.mode, 0, 0, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C#%d driver install failed. Error: %s", i2c->def->i2cPort, esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "I2C#%d driver installed successfully", i2c->def->i2cPort);
    i2c->is_initialized = true;

    return ESP_OK;
}

// <-- NUEVA FUNCIÓN: Implementación de la transferencia
bool i2cdrvMessageTransfer(I2cDrv *i2c, I2cMessage *message)
{
    if (!i2c->is_initialized || i2c->isBusFreeMutex == NULL) {
        ESP_LOGE(TAG, "I2C#%d not initialized.", i2c->def->i2cPort);
        message->status = i2cNack;
        return false;
    }

    // Proteger el acceso al bus I2C de otras tareas
    if (xSemaphoreTake(i2c->isBusFreeMutex, pdMS_TO_TICKS(100)) == pdFALSE) {
        ESP_LOGE(TAG, "I2C#%d bus busy.", i2c->def->i2cPort);
        message->status = i2cNack;
        return false;
    }

    esp_err_t err = ESP_FAIL;
    message->status = i2cNack; // Asumir fallo por defecto

    // Bucle para reintentos
    for (int i = 0; i <= message->nbrOfRetries; i++) {
        if (message->direction == i2cWrite) {
            err = i2c_master_write_to_device(
                i2c->def->i2cPort,
                message->slaveAddress,
                message->buffer,
                message->messageLength,
                pdMS_TO_TICKS(100) // Timeout
            );
        } else { // i2cRead
            err = i2c_master_read_from_device(
                i2c->def->i2cPort,
                message->slaveAddress,
                message->buffer,
                message->messageLength,
                pdMS_TO_TICKS(100) // Timeout
            );
        }

        if (err == ESP_OK) {
            message->status = i2cAck;
            break; // Salir del bucle si la operación fue exitosa
        }

        if (i < message->nbrOfRetries) {
            vTaskDelay(pdMS_TO_TICKS(5)); // Pequeña espera antes de reintentar
        }
    }
    
    // Liberar el mutex para que otras tareas puedan usar el bus
    xSemaphoreGive(i2c->isBusFreeMutex);

    if (err != ESP_OK) {
        ESP_LOGD(TAG, "I2C transfer failed on port %d, addr 0x%02X. Error: %s",
                 i2c->def->i2cPort, message->slaveAddress, esp_err_to_name(err));
    }
    
    return (message->status == i2cAck);
}


// <-- NUEVA FUNCIÓN: Implementación de la creación de mensajes
void i2cdrvCreateMessage(I2cMessage *message,
                         uint8_t  slaveAddress,
                         I2cDirection  direction,
                         uint32_t length,
                         uint8_t  *buffer)
{
    memset(message, 0, sizeof(I2cMessage));
    message->slaveAddress = slaveAddress;
    message->direction = direction;
    message->messageLength = length;
    message->buffer = buffer;
    message->internalAddress = I2C_NO_INTERNAL_ADDRESS;
    message->nbrOfRetries = 1; // Un reintento por defecto
    message->status = i2cNack;
}

// <-- NUEVA FUNCIÓN: Implementación de la creación de mensajes con dirección interna
void i2cdrvCreateMessageIntAddr(I2cMessage *message,
                                uint8_t  slaveAddress,
                                bool isInternal16,
                                uint16_t intAddress,
                                I2cDirection  direction,
                                uint32_t length,
                                uint8_t  *buffer)
{
    i2cdrvCreateMessage(message, slaveAddress, direction, length, buffer);
    message->isInternal16bit = isInternal16;
    message->internalAddress = intAddress;
}