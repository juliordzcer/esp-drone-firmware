

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "stm32_legacy.h"
#include "i2c_drv.h"
#include "config.h"
#define DEBUG_MODULE "I2CDRV"
#include "debug_cf.h"

// Definitions of sensors I2C bus
#define I2C_DEFAULT_SENSORS_CLOCK_SPEED             400000

// Definition of eeprom and deck I2C buss,use two i2c with 400Khz clock simultaneously could trigger the watchdog
#define I2C_DEFAULT_DECK_CLOCK_SPEED                100000

static bool isinit_i2cPort[2] = {0, 0};

// Cost definitions of busses
static const I2cDef sensorBusDef = {
    .i2cPort            = I2C_NUM_0,
    .i2cClockSpeed      = I2C_DEFAULT_SENSORS_CLOCK_SPEED,
    .gpioSCLPin         = CONFIG_I2C0_PIN_SCL,
    .gpioSDAPin         = CONFIG_I2C0_PIN_SDA,
    .gpioPullup         = GPIO_PULLUP_DISABLE,
};

I2cDrv sensorsBus = {
    .def                = &sensorBusDef,
};

static const I2cDef deckBusDef = {
    .i2cPort            = I2C_NUM_1,
    .i2cClockSpeed      = I2C_DEFAULT_DECK_CLOCK_SPEED,
    .gpioSCLPin         = CONFIG_I2C1_PIN_SCL,
    .gpioSDAPin         = CONFIG_I2C1_PIN_SDA,
    .gpioPullup         = GPIO_PULLUP_ENABLE,
};

I2cDrv deckBus = {
    .def                = &deckBusDef,
};

static void i2cdrvInitBus(I2cDrv *i2c)
{
    if (isinit_i2cPort[i2c->def->i2cPort]) {
        return;
    }

    i2c_config_t conf = {0};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = i2c->def->gpioSDAPin;
    conf.sda_pullup_en = i2c->def->gpioPullup;
    conf.scl_io_num = i2c->def->gpioSCLPin;
    conf.scl_pullup_en = i2c->def->gpioPullup;
    conf.master.clk_speed = i2c->def->i2cClockSpeed;
    esp_err_t err = i2c_param_config(i2c->def->i2cPort, &conf);

    if (!err) {
        err = i2c_driver_install(i2c->def->i2cPort, conf.mode, 0, 0, 0);
    }

    DEBUG_PRINTI(" i2c %d driver install return = %d", i2c->def->i2cPort, err);
    i2c->isBusFreeMutex = xSemaphoreCreateMutex();
    isinit_i2cPort[i2c->def->i2cPort] = true;
}


//-----------------------------------------------------------

void i2cdrvInit(I2cDrv *i2c)
{
    i2cdrvInitBus(i2c);
}

void i2cdrvTryToRestartBus(I2cDrv *i2c)
{
    i2cdrvInitBus(i2c);
}

