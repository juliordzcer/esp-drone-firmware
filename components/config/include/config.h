/**
 *
 * ESP-Drone Firmware
 *
 * Copyright 2019-2020  Espressif Systems (Shanghai)
 * Copyright (C) 2011-2012 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * config.h - Main configuration file
 *
 * This file define the default configuration of the copter
 * It contains two types of parameters:
 * - The global parameters are globally defined and independent of any
 * compilation profile. An example of such define could be some pinout.
 * - The profiled defines, they are parameter that can be specific to each
 * dev build. The vanilla build is intended to be a "customer" build without
 * fancy spinning debugging stuff. The developers build are anything the
 * developer could need to debug and run his code/crazy stuff.
 *
 * The golden rule for the profile is NEVER BREAK ANOTHER PROFILE. When adding a
 * new parameter, one shall take care to modified everything necessary to
 * preserve the behavior of the other profiles.
 *
 * For the flag. T_ means task. H_ means HAL module. U_ would means utils.
 */

#ifndef CONFIG_H_
#define CONFIG_H_

#include "usec_time.h"
#include "sdkconfig.h"

#define PROTOCOL_VERSION 4
#define QUAD_FORMATION_X // Define para la formación del cuadricóptero (ej. forma de 'X')

// Comprobaciones de la configuración del objetivo de hardware
#ifdef CONFIG_TARGET_ESPLANE_V2_S2
#ifndef CONFIG_IDF_TARGET_ESP32S2
#error "ESPLANE_V2 hardware with ESP32S2 onboard" // Error si el hardware es ESP-Plane V2 pero el objetivo no es ESP32S2
#endif
#elif defined(CONFIG_TARGET_ESPLANE_V1)
#ifndef CONFIG_IDF_TARGET_ESP32
#error "ESPLANE_V1 hardware with ESP32 onboard" // Error si el hardware es ESP-Plane V1 pero el objetivo no es ESP32
#endif
#elif defined(CONFIG_TARGET_ESP32_S2_DRONE_V1_2)
#ifdef CONFIG_IDF_TARGET_ESP32
#error "ESP32_S2_DRONE_V1_2 hardware with ESP32S2/S3 onboard" // Error si el hardware es ESP32-S2 Drone V1.2 pero el objetivo es ESP32 (debería ser S2/S3)
#endif
#endif

// Prioridades de las tareas. Un número más alto indica mayor prioridad.
// Tareas de estado del sistema
#define SYSTEM_TASK_PRI         1
#define PM_TASK_PRI             1
#define LEDSEQCMD_TASK_PRI      1
// Tareas de comunicación TX
#define UDP_TX_TASK_PRI         2
#define CRTP_TX_TASK_PRI        2
// Tareas de comunicación RX
#define UDP_RX_TASK_PRI         2
#define EXTRX_TASK_PRI          2
#define UART2_TASK_PRI          2
#define SYSLINK_TASK_PRI        2
#define USBLINK_TASK_PRI        2
#define WIFILINK_TASK_PRI       2
#define CRTP_RX_TASK_PRI        2
#define CMD_HIGH_LEVEL_TASK_PRI 3
#define INFO_TASK_PRI           2
#define LOG_TASK_PRI            2
#define MEM_TASK_PRI            2
#define PARAM_TASK_PRI          2
// Tareas relacionadas con sensores y estabilización
// #define PROXIMITY_TASK_PRI      5
// #define FLOW_TASK_PRI           5
// #define ZRANGER2_TASK_PRI       5
// #define ZRANGER_TASK_PRI        5
#define SENSORS_TASK_PRI        6
#define STABILIZER_TASK_PRI     7
#define KALMAN_TASK_PRI         4

// El filtro Kalman consume mucha CPU. Para sistemas de un solo núcleo, se necesita bajar la prioridad.
#if CONFIG_FREERTOS_UNICORE
  #undef KALMAN_TASK_PRI
  #define KALMAN_TASK_PRI         1
#endif

// Nombres de las tareas
#define CMD_HIGH_LEVEL_TASK_NAME "CMDHL"
#define CRTP_RX_TASK_NAME       "CRTP-RX"
#define CRTP_TX_TASK_NAME       "CRTP-TX"
#define EXTRX_TASK_NAME         "EXTRX"
// #define FLOW_TASK_NAME          "FLOW"
#define KALMAN_TASK_NAME        "KALMAN"
#define LEDSEQCMD_TASK_NAME     "LEDSEQCMD"
#define LOG_TASK_NAME           "LOG"
#define MEM_TASK_NAME           "MEM"
#define PARAM_TASK_NAME         "PARAM"
#define PM_TASK_NAME            "PWRMGNT"
// #define PROXIMITY_TASK_NAME     "PROXIMITY"
#define SENSORS_TASK_NAME       "SENSORS"
#define STABILIZER_TASK_NAME    "STABILIZER"
#define SYSLINK_TASK_NAME       "SYSLINK"
#define SYSTEM_TASK_NAME        "SYSTEM"
#define UART2_TASK_NAME         "UART2"
#define UDP_RX_TASK_NAME        "UDP_RX"
#define UDP_TX_TASK_NAME        "UDP_TX"
#define USBLINK_TASK_NAME       "USBLINK"
#define WIFILINK_TASK_NAME      "WIFILINK"
// #define ZRANGER2_TASK_NAME      "ZRANGER2"
// #define ZRANGER_TASK_NAME       "ZRANGER"

// Tamaños de pila de las tareas
#define configBASE_STACK_SIZE CONFIG_BASE_STACK_SIZE
#define CMD_HIGH_LEVEL_TASK_STACKSIZE (2 * configBASE_STACK_SIZE)
#define CRTP_RX_TASK_STACKSIZE        (3 * configBASE_STACK_SIZE)
#define CRTP_TX_TASK_STACKSIZE        (3 * configBASE_STACK_SIZE)
#define EXTRX_TASK_STACKSIZE          (1 * configBASE_STACK_SIZE)
// #define FLOW_TASK_STACKSIZE           (3 * configBASE_STACK_SIZE)
#define KALMAN_TASK_STACKSIZE         (3 * configBASE_STACK_SIZE)
#define LEDSEQCMD_TASK_STACKSIZE      (2 * configBASE_STACK_SIZE)
#define LOG_TASK_STACKSIZE            (3 * configBASE_STACK_SIZE)
#define MEM_TASK_STACKSIZE            (2 * configBASE_STACK_SIZE)
#define PARAM_TASK_STACKSIZE          (2 * configBASE_STACK_SIZE)
#define PM_TASK_STACKSIZE             (4 * configBASE_STACK_SIZE)
#define SENSORS_TASK_STACKSIZE        (5 * configBASE_STACK_SIZE)
#define STABILIZER_TASK_STACKSIZE     (5 * configBASE_STACK_SIZE)
#define SYSLINK_TASK_STACKSIZE        (1 * configBASE_STACK_SIZE)
#define SYSTEM_TASK_STACKSIZE         (6 * configBASE_STACK_SIZE)
#define UART2_TASK_STACKSIZE          (1 * configBASE_STACK_SIZE)
#define UDP_RX_TASK_STACKSIZE         (4 * configBASE_STACK_SIZE)
#define UDP_TX_TASK_STACKSIZE         (4 * configBASE_STACK_SIZE)
#define USBLINK_TASK_STACKSIZE        (1 * configBASE_STACK_SIZE)
#define WIFILINK_TASK_STACKSIZE       (4 * configBASE_STACK_SIZE)
// #define ZRANGER2_TASK_STACKSIZE       (4 * configBASE_STACK_SIZE)
// #define ZRANGER_TASK_STACKSIZE        (2 * configBASE_STACK_SIZE)

// Parámetros de radio
// La tasa de radio. 2Mbit/s.
#define RADIO_RATE_2M 2
// El canal de radio. De 0 a 125.
#define RADIO_CHANNEL 80
// La tasa de datos de radio.
#define RADIO_DATARATE RADIO_RATE_2M
// La dirección de radio (64 bits).
#define RADIO_ADDRESS 0xE7E7E7E7E7ULL

/**
 * \def PROPELLER_BALANCE_TEST_THRESHOLD
 * Este es el umbral para que una hélice/motor pase la prueba. Calcula la varianza del acelerómetro X+Y
 * cuando la hélice está girando.
 */
#define PROPELLER_BALANCE_TEST_THRESHOLD  2.5f

#endif /* CONFIG_H_ */