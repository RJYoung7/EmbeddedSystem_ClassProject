/*
Citation:
Code to enable and disable light referenced from Blinky.c from StellarisWare example package.
*/

#include "inc/lm3s8962.h"
#include "warningAlarm.h"
#include "dataPtrs.c"
#include <stdlib.h>
#include <stdio.h>
#include "bool.h"
#include "systemTimeBase.h"
#include "driverlib/pwm.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/sysctl.h"
#include "Flags.h"
#include "FreeRTOS.h"
#include "task.h"

/*
Function alarm
Input: pointer to alarmData
Output Null
Do: Checks if vitals are out of range
*/
void alarm(void *data)
{
  for( ;; )
  {
  warningAlarmData2 * alarm = (warningAlarmData2*) data;
  
  // Call functions
  checkWarnings(data);
  annunciate(data);
  //auralAnnunciate(data);
  
   // Add serial to queue if warning has occurred
  if((*(alarm->tempHighPtr)) || (*(alarm->pulseLowPtr)) || (*(alarm->bpHighPtr)))
  {
    serialFlag = 1;
  }
  vTaskDelay(10);
  }
  //return;
}

/*
Function checkWarnings
Input pointer to alarmData
Output Null
Do: Checks raw measurements against ranges and annunciates accordingly
*/

void checkWarnings(void *data)
{
  warningAlarmData2 * alarm = (warningAlarmData2*) data;
  
  //find the current index of the array based on call count. 
  unsigned int index = ((*(alarm->countCallsPtr)) % 8);
  
  // Get data from structs to use in functions
  unsigned int* tempBuf = (*alarm).temperatureRawBufPtr;
  unsigned int* bpBuf = (*alarm).bloodPressRawBufPtr;
  unsigned int* pulseBuf = (*alarm).pulseRateRawBufPtr;
  unsigned char* bpOut = (*alarm).bpOutOfRangePtr;
  unsigned char* tempOut = (*alarm).tempOutOfRangePtr;
  unsigned char* pulseOut = (*alarm).pulseOutOfRangePtr;
  Bool* bpHigh = (*alarm).bpHighPtr;
  Bool* tempHigh = (*alarm).tempHighPtr;
  Bool* pulseLow = (*alarm).pulseLowPtr;

  // Check vitals against prescribed ranges. Set warnings accordingly
  checkTemp(tempBuf, tempHigh, index, tempOut);
  checkBp(bpBuf, bpHigh, index, bpOut);
  checkPulse(pulseBuf, pulseLow, index, pulseOut);
  return;
}



/*
Function checkTemp
Input: pointer to temperatureRaw, pointer to tempHigh
Output: Null
Do: Checks if values are within normal range and sets bool accordingly.
*/
void checkTemp(unsigned int* temp, Bool* tempHigh, int index, unsigned char* tempOut)
{
  // Check if temperature is in range. Set warning accordingly
  if((temp[index]) < 36.1 || (temp[index]) > 37.8)
  {
    *tempHigh = TRUE;
    *tempOut = 89;
  } 
  else
  {
    *tempHigh = FALSE;
    *tempOut = 78;
  } 
}

/*
Function checkBp
Input: pointer to systolicRaw, pointer to diastolicRaw, pointer to bpHigh
Output: Null
Do: Checks if values are within normal range and sets bool accordingly.
*/
void checkBp(unsigned int* bpBuf, Bool* bpHigh, int index, unsigned char* bpOut)
{
  // Check if blood pressure is in range.  Set warnings accordingly
  if ((bpBuf[index]) > 60.5 || (bpBuf[index]) < 55.5 || (bpBuf[index + 8]) > 49.33 || (bpBuf[index + 8]) < 42.67)
  {
    *bpHigh = TRUE; 
    *bpOut = 89;
  }
  else
  {
    *bpHigh = FALSE;
    *bpOut = 78;
  }
}

/*
Function checkPulse
Input: pointer to pulseRateRaw, pointer to pulseLow
Output: Null
Do: Checks if values are within normal range and sets bool accordingly.
*/
void checkPulse(unsigned int* pulse, Bool* pulseLow, int index, unsigned char* pulseOut)
{
  // Check if pulse rate is in range. Set warning accordingly.
  if ((int)(*pulse) < 60)
  {
    *pulseLow = TRUE;
    *pulseOut = 89;
  }
  else
  {
    *pulseLow = FALSE;
    *pulseOut = 78;
  }
}

/*
Function auralAnnunciate
Input: warning data
Output: Sound
Do: Checks if there is a warning and creates aural annunciation
*/
void auralAnnunciate(void *data)
{
  warningAlarmData2 * alarm = (warningAlarmData2*) data;
  

  // If warning true
  if((*(alarm->tempHighPtr)) || (*(alarm->pulseLowPtr)) || (*(alarm->bpHighPtr)))
  {
    // Enable if 5 seconds have elapsed since last button press
    if(auralFlag == 0 && (globalCounter - auralCounter >= 5000))
    {
      auralFlag = 1;
      PWMGenEnable(PWM_BASE, PWM_GEN_0);
    }
  } 
  // Disable if no warning present
  else
  {
    if(auralFlag == 1)
    {
      auralFlag = 0;
      PWMGenDisable(PWM_BASE, PWM_GEN_0);
    }
  }
  
}

/*
Function: annunciate
Input: warning data
Output: Flashing of LED on board
Do: Flashes LED at rate per specific warning.
*/
void annunciate(void *data)
{
  // Declare static variables 
  static int led0 = 0;          // Flag for led0
  static int led1 = 0;          // Flag for led1
  static int led2 = 0;          // Flag for led3
  static long led1Count = 0;    // Counter for led1
  static long led2Count = 0;    // Counter for led2
  static long pwmCounter = 0;   // Counter for pwm
  static int pwm1 = 0;          // Flag for pwm
  
  // Get data from struct
  warningAlarmData2 * alarm = (warningAlarmData2*) data;
  unsigned long* previousCount = (*alarm).previousCountPtr;
  const long pulseFlash = *(alarm->pulseFlashPtr);
  const long tempFlash = *(alarm->tempFlashPtr);
  const long bpFlash = *(alarm->bpFlashPtr);
  unsigned int index = ((*(alarm->countCallsPtr)) % 8);
  unsigned int* bpBuf = (*alarm).bloodPressRawBufPtr;
  
  // Flash at the correct rate for each warning.
  //Pulse
  if(*(alarm->tempHighPtr))//pulseLowPtr
  { 
    if(globalCounter - (*previousCount) >= tempFlash)//pulseFlash
    {
      
      (*previousCount) = globalCounter;
      if((led0) == 1)
      {
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, 0x00);
        (led0) = 0;
      }
      else
      {
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, 0x01);
        (led0) = 1;
      }
    }    
  }
  //Temp
  else if (*(alarm->pulseLowPtr))//tempHighPtr
  {
    if(globalCounter - (*previousCount) >= pulseFlash)//tempFlash
    { 
      (*previousCount) = globalCounter;
      if((led0) == 1)
      {
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, 0x00);
        (led0) = 0;
      }
      else
      {
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, 0x01);
        (led0) = 1;
      }
    }
  }
  else if (!(*(alarm->pulseLowPtr)) && !(*(alarm->tempHighPtr)))
  {
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, 0x01);
    (led0) = 1;
  }
    
  // Flash
  if (*(alarm->bpHighPtr))
  {
    if(globalCounter - (led1Count) >= bpFlash)
    {
      (led1Count) = globalCounter;
      if((led1) == 1)
      {
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0x04);
        (led1) = 0;
      }
      else
      {
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0x00);
        (led1) = 1;
      }
    }
  }
  else if (!(*(alarm->bpHighPtr)))
  {
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0x00);
    (led1) = 1;
  }
    
  /* LED2 - Systolic Blood Pressure Alarm*/
  if (*(alarm->bpHighPtr) || (bpBuf[index] >= 73.5))
  {
    // Initial warning/alarm.  Enables alarm if over 20% normal range
    if(auralFlag == 0 && bpBuf[index] >= 73.5 && ackFlag == 0)
    { 
          PWMGenEnable(PWM_BASE, PWM_GEN_0);
          GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0x00);
          pwm1 = 1;
          auralFlag = 1;
    }
    // 1 second pulse of alarm until acknowledged.
    else if(auralFlag == 1 && bpBuf[index] >= 73.5 && ackFlag == 0)
    { 
      if(globalCounter - pwmCounter >= tempFlash)
      {
        pwmCounter = globalCounter;
        if(pwm1 == 0)
        {
          PWMGenEnable(PWM_BASE, PWM_GEN_0);
          pwm1 = 1;
        }
        else
        {
          PWMGenDisable(PWM_BASE, PWM_GEN_0);
          pwm1 = 0;
        }
      }
    }
    /* Alarm has been manually disabled.  Flash 1 second continues until
      within 110% of normal range */
    else if (auralFlag == 0 && bpBuf[index] >=67 && ackFlag == 1)
    {
      
      if(globalCounter - (led2Count) >= tempFlash)
      {
        led2Count = globalCounter;
        if((led2) == 1)
        {
          GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0x08);
          led2 = 0;
        }
        else
        {
          GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0x00);
          led2 = 1;
        }
      }
    }
    // If alarm is on and sys pressure falls below 120%
    else if (auralFlag == 1 && bpBuf[index] >=67 && ackFlag == 0)
    {
      PWMGenDisable(PWM_BASE, PWM_GEN_0);
      auralFlag = 0;
    }
    // Flash 1 second when between 110 and 120%
    else if (auralFlag == 0 && bpBuf[index] >=67 && ackFlag == 0)
    {
      if(globalCounter - (led2Count) >= tempFlash)
      {
        led2Count = globalCounter;
        if((led2) == 1)
        {
          GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0x08);
          led2 = 0;
        }
        else
        {
          GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0x00);
          led2 = 1;
        }
      }
    }
  }
  // Systolic blood pressure in normal range.  Solid light
  else if (!(*(alarm->bpHighPtr)))
  {
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0x00);
    led2 = 1;
    PWMGenDisable(PWM_BASE, PWM_GEN_0);
    auralFlag = 0;
  }
    
  return; 
}

/*
Function enableVisibleAnnunciation
Input: N/A
Output: Null
Do: Turns on LED on StellarisWare board
*/
void enableVisibleAnnunciation()
{
//  GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, 1);
//  GPIOPinWrite(GPIO_PORTF_BASE, 0x04, 1);
//  GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0x01);
//  GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_1, 0x01);
  //GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, 0x01);
  
  return;
}

/*
Function disableVisibleAnnunciation
Input: N/A
Output: Null
Do: Turns off LED on StellarisWare board
*/

void disableVisibleAnnunciation()
{
//  GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, 0);
//  GPIOPinWrite(GPIO_PORTF_BASE, 0x04,0);
//  GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0x00);
//  GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_1, 0x00);
  
  GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, 0x00);
  return;
}

