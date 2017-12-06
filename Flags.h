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
extern unsigned int tempFlag;
extern unsigned int diaFlag;
extern unsigned int sysFlag;
extern unsigned int pulseFlag;
extern TaskHandle_t xComputeHandle;
extern TaskHandle_t xEKGHandle;
extern TaskHandle_t xDisplayHandle;

#endif