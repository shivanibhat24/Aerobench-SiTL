#ifndef MAIN_H
#define MAIN_H

/* STM32 HAL */
#include "stm32g0xx_hal.h"

/* Standard C */
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

/* FreeRTOS / CMSIS-RTOS2 */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"
#include "cmsis_os.h"

/* Project headers */
#include "sensor_data.h"
#include "protocol.h"
#include "blackbox.h"

/* ------------------------------------------------------------------ */
/*  Global HAL handles (defined in main.c)                             */
/* ------------------------------------------------------------------ */

extern UART_HandleTypeDef  huart2;
extern IWDG_HandleTypeDef  hiwdg;
extern CRC_HandleTypeDef   hcrc;

/* ------------------------------------------------------------------ */
/*  Global FreeRTOS IPC objects (defined in main.c)                    */
/* ------------------------------------------------------------------ */

extern osMessageQueueId_t  g_sensor_queue;
extern osMutexId_t         g_state_mutex;
extern osEventFlagsId_t    g_fault_flags;
extern blackbox_t          g_blackbox;

/* ------------------------------------------------------------------ */
/*  Error handler                                                       */
/* ------------------------------------------------------------------ */

void Error_Handler(void);

#endif /* MAIN_H */
