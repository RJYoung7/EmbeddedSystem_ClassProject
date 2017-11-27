#ifndef FLAGS_H_
#define FLAGS_H_

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

extern unsigned long auralFlag;
extern unsigned long ackFlag;
extern unsigned int auralCounter;
extern unsigned long computeFlag;
extern unsigned int ekgCounter;
extern unsigned long serialFlag;
extern TaskHandle_t xComputeHandle;
extern TaskHandle_t xEKGHandle;
extern TaskHandle_t xDisplayHandle;
extern SemaphoreHandle_t xSemaphore;

#endif