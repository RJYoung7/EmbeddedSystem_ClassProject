#ifndef FLAGS_H_
#define FLAGS_H_

#include "FreeRTOS.h"
#include "task.h"

extern unsigned long auralFlag;
extern unsigned int auralCounter;
extern unsigned long computeFlag;
extern unsigned long serialFlag;
extern TaskHandle_t xComputeHandle;

#endif