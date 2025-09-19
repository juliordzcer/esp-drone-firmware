#ifndef I2C_H // <-- CAMBIO: Nombre del header guard más específico
#define I2C_H

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "driver/i2c.h"

#include "stm32_legacy.h"

#define I2C_NO_INTERNAL_ADDRESS   0xFFFF

typedef enum {
    i2cAck,
    i2cNack
} I2cStatus;

typedef enum {
    i2cWrite,
    i2cRead
} I2cDirection;

/**
 * Structure used to capture the I2C message details.
 */
typedef struct { // <-- CAMBIO: Renombrado de _I2cMessage a I2cMessage para consistencia
    uint32_t        messageLength;      //< How many bytes of data to send or received.
    uint8_t         slaveAddress;       //< The slave address of the device on the I2C bus.
    uint8_t         nbrOfRetries;       //< Number of retries on failure (NACK).
    I2cDirection    direction;          //< Direction of message
    I2cStatus       status;             //< i2c status
    xQueueHandle    clientQueue;        //< (Optional) Queue to send received messages to.
    bool            isInternal16bit;    //< Is internal address 16 bit. If false 8 bit.
    uint16_t        internalAddress;    //< Internal address of device. Use I2C_NO_INTERNAL_ADDRESS if none.
    uint8_t         *buffer;            //< Pointer to the buffer for data transfer.
} I2cMessage;

typedef struct {
    i2c_port_t      i2cPort;
    uint32_t        i2cClockSpeed;
    gpio_num_t      gpioSCLPin;         // <-- CAMBIO: Usar tipo gpio_num_t del ESP-IDF
    gpio_num_t      gpioSDAPin;         // <-- CAMBIO: Usar tipo gpio_num_t del ESP-IDF
    gpio_pullup_t   gpioPullup;
} I2cDef;

typedef struct {
    const I2cDef    *def;               //< Definition of the i2c
    SemaphoreHandle_t isBusFreeMutex;   //< Mutex to protect bus access
    bool            is_initialized;     //< <-- CAMBIO: Flag de inicialización encapsulado
} I2cDrv;

// Definitions of i2c busses found in c file.
extern I2cDrv deckBus;
extern I2cDrv sensorsBus;

/**
 * Initialize i2c peripheral as defined by its definition struct.
 * This function is safe to call multiple times.
 *
 * @param i2c The I2C driver instance to initialize.
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t i2cdrvInit(I2cDrv *i2c); // <-- CAMBIO: Ahora devuelve esp_err_t

/**
 * Send or receive a message over the I2C bus.
 * This function is synchronous and thread-safe.
 *
 * @param i2c      i2c bus to use.
 * @param message  An I2cMessage struct containing all the i2c message
 * information. The message status will be updated.
 * @return         true if successful (ACK), false otherwise (NACK).
 */
bool i2cdrvMessageTransfer(I2cDrv *i2c, I2cMessage *message);

/**
 * Create a simple message to transfer (without an internal register address).
 *
 * @param message       pointer to message struct that will be filled in.
 * @param slaveAddress  i2c slave address
 * @param direction     i2cWrite or i2cRead
 * @param length        Length of message
 * @param buffer        pointer to buffer of send/receive data
 */
void i2cdrvCreateMessage(I2cMessage *message,
                         uint8_t  slaveAddress,
                         I2cDirection  direction,
                         uint32_t length,
                         uint8_t  *buffer);

/**
 * Create a message to transfer with an internal "register" address.
 *
 * @param message       pointer to message struct that will be filled in.
 * @param slaveAddress  i2c slave address
 * @param isInternal16  true for 16-bit reg address, false for 8-bit.
 * @param intAddress    The internal register address to access.
 * @param direction     i2cWrite or i2cRead
 * @param length        Length of message
 * @param buffer        pointer to buffer of send/receive data
 */
void i2cdrvCreateMessageIntAddr(I2cMessage *message,
                                uint8_t  slaveAddress,
                                bool isInternal16,
                                uint16_t intAddress,
                                I2cDirection  direction,
                                uint32_t length,
                                uint8_t  *buffer);

#endif // I2C_H