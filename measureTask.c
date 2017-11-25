// Main and measureTask
#include <stdio.h>
#include "measureTask.h"
#include "dataPtrs.h"
#include "bool.h"
#include "systemTimeBase.h"
#include "Flags.h"
#include "FreeRTOS.h"
#include "task.h"
#include "adc.h"
#include "inc/hw_memmap.h"


void measure(void* data)
{
  for( ;; )
  {
    
//    unsigned long ulADC0_Value[1];
//
//    //
//    // These variables are used to store the temperature conversions for
//    // Celsius and Fahrenheit.
//    //
//    volatile unsigned long ulTemp_ValueC;
//    volatile unsigned long ulTemp_ValueF;
//    //
//    // Sample the temperature sensor forever.  Display the value on the
//    // console.
//    //
//    
//    
//
//        while(1)
//    {
//        //
//        // Trigger the ADC conversion.
//        //
//        ADCProcessorTrigger(ADC0_BASE, 3);
//
//        //
//        // Wait for conversion to be completed.
//        //
//        while(!ADCIntStatus(ADC0_BASE, 3, false))
//        {
//        }
//
//        //
//        // Read ADC Value.
//        //
//        ADCSequenceDataGet(ADC0_BASE, 3, ulADC0_Value);
//
//        //
//        // Use non-calibrated conversion provided in the data sheet.  Make
//        // sure you divide last to avoid dropout.
//        //
//        ulTemp_ValueC = (long)(147.5 - ((225 * ulADC0_Value[0]) / 1023));
//
//        //
//        // Get fahrenheit value.  Make sure you divide last to avoid dropout.
//        //
//        ulTemp_ValueF = ((ulTemp_ValueC * 9) + 160) / 5;
//
//        //
//        // Display the temperature value on the console.
//        //
////        UARTprintf("Temperature = %3d*C or %3d*F\r", ulTemp_ValueC,
////                   ulTemp_ValueF);
//
//        //
//        // This function provides a means of generating a constant length
//        // delay.  The function delay (in cycles) = 3 * parameter.  Delay
//        // 250ms arbitrarily.
//        //
//        SysCtlDelay(SysCtlClockGet() / 12);
//    }
    measureData2 * measureDataPtr = (measureData2*) data;
 
    //xTaskCreate(measureTempArray, "Measure Temp", 500, (void*)data, 1, &xTempHandle);
    measureTempArray(data);
    measureSysBPArray(data);
    measureDiaBPArray(data);
    measurePRArray(data);

    /*Moved this to after the measurements so we start at index 0
    increment the count entry */
    ++(*(*measureDataPtr).countCallsPtr);
    
    vTaskResume(xComputeHandle);
    
    // Delay for 5 seconds
    vTaskDelay(5000);
    
  }
}

/*
Function measureTempArray
Input pointer to measureData
Output Null
Do: updates the tempRaw based on algorithm
*/
void measureTempArray(void* data){
  measureData2* measureDataPtr = (measureData2*) data;
  
  //Creates a local pointer to the countCalls Variable
  unsigned int* countCalls = (*measureDataPtr).countCallsPtr;
  
  //Creates a local pointer to the start of the array
  unsigned int* tempRawBuf = (*measureDataPtr).temperatureRawBufPtr;
  
  //find the current index of the array based on call count. 
  unsigned int next = (*countCalls +1) %8;

  /* This array is used for storing the data read from the ADC FIFO. It
   must be as large as the FIFO for the sequencer in use.  This example
   uses sequence 3 which has a FIFO depth of 1.  If another sequence
   was used with a deeper FIFO, then the array size must be changed. */
  unsigned long ulADC0_Value[1];

  // Trigger the ADC conversion.
  ADCProcessorTrigger(ADC0_BASE, 3);

  // Wait for conversion to be completed.
  while(!ADCIntStatus(ADC0_BASE, 3, false))
  {
  }

  /* Clear the interrupt status flag.  This is done to make sure the
  interrupt flag is cleared before we sample. */
  ADCIntClear(ADC0_BASE, 3);

  // Read ADC Value.
  ADCSequenceDataGet(ADC0_BASE, 3, ulADC0_Value);

  // Use non-calibrated conversion provided in the data sheet.
  tempRawBuf[next] = (int)(147.5 - ((225 * ulADC0_Value[0]) / 1023));
};

/*
Function measureSysBp
Input pointer to measureData
Output Null
Do: Places Systolic into array indexes 0-7
*/
void measureSysBPArray(void* data){
    measureData2* measureDataPtr = (measureData2*) data;
    //printf("This is a measureSysBp Function \n");
    //Check to see if the DiaBp is complete and repeat the original proces
    unsigned int* countCalls = (*measureDataPtr).countCallsPtr;
    unsigned int* bloodPressRawBuf  = (*measureDataPtr).bloodPressRawBufPtr;
    unsigned int* sysComplete = (*measureDataPtr).sysCompletePtr;
    unsigned int* diaComplete = (*measureDataPtr).diaCompletePtr;
    //find the current index of the array based on call count. 
    unsigned int sysLast = (*countCalls) %8;
    unsigned int sysNext = (*countCalls +1) %8;
    
    if (1==*diaComplete && bloodPressRawBuf[sysLast]>100){
      bloodPressRawBuf[sysNext] = 80;
      *diaComplete = 0;
    }
    // If the sysBP <= 100 its not complete so we increment it
    if (100 >= bloodPressRawBuf[sysLast] ){
      if  ( (*countCalls % 2) == 0){
        bloodPressRawBuf[sysNext] = bloodPressRawBuf[sysLast] + 3;
      }
      else{
        bloodPressRawBuf[sysNext] = bloodPressRawBuf[sysLast] - 1;
      }
    }
    // If sysBP > 100 it is complete and we wait til diaCompletes
    if (100 < bloodPressRawBuf[sysNext]){
      *sysComplete = 1;
    }
};

/*
Function measureDiaBp
Input pointer to measureData
Output Null
Do: Places Systolic into array indexes 8-15
*/
void measureDiaBPArray(void* data){
  
    measureData2* measureDataPtr = (measureData2*) data;
    unsigned int* countCalls = (*measureDataPtr).countCallsPtr;
    unsigned int* bloodPressRawBuf = (*measureDataPtr).bloodPressRawBufPtr;
    unsigned int* sysComplete = (*measureDataPtr).sysCompletePtr;
    unsigned int* diaComplete = (*measureDataPtr).diaCompletePtr;
    unsigned int diaLast = ((*countCalls) %8) + 8;
    unsigned int diaNext = ((*countCalls +1) %8) + 8;
   // printf("This is a measureSysBp Function \n");
  //Check to see if the DiaBp is complete and repeat the original proces
     if (1==*sysComplete && bloodPressRawBuf[diaLast]<40){
      bloodPressRawBuf[diaNext] = 80;
      *sysComplete = 0;
      }
    // If diastolyic BP is above 40 it is not complete
    if (40 <= bloodPressRawBuf[diaLast]){
      if  ( ((*countCalls) % 2) == 0){
        bloodPressRawBuf[diaNext] = bloodPressRawBuf[diaLast] - 2;
      }
      else{
        bloodPressRawBuf[diaNext] = bloodPressRawBuf[diaLast] + 1;
      }
    } 
    // diastolyic BP drops below 40 and is complete
    if (40 > bloodPressRawBuf[diaNext]){
      *diaComplete = 1;
    }
};


/*
Function measurePrArray
Input pointer to measureData
Output Null
Do: Needs to be updated with the model transducer handling.
*/
void measurePRArray(void* data){
 int check=0;
    int beatCount=0;
    int bpm=0;
    int change =0;
    measureData2* measureDataPtr = (measureData2*) data;
    unsigned int clock = globalCounter;//(*measureDataPtr).globalCounterPtr;
    unsigned int temp=clock;
    while(clock<(temp+100)){
        
        clock=globalCounter;//(*measureDataPtr).globalCounterPtr;
        unsigned long* beat=(*measureDataPtr).prPtr;
        
        if(*beat==1 && check==0){
            beatCount++;
            check=1;
        }
        else if(*beat==0){
           check=0;
        }
    }
    
    bpm=(beatCount*60/3);
    
    unsigned int* countCalls = (*measureDataPtr).countCallsPtr;
    unsigned int* pulseRateRawBuf = (*measureDataPtr).pulseRateRawBufPtr;
    unsigned int prLast = (*countCalls) %8;
    unsigned int prNext = (*countCalls+1) %8;
    //Check to see if we prLast would cause divide by 0
    if(pulseRateRawBuf[prLast]!=0){
      change = ((pulseRateRawBuf[prLast]-bpm)*100)/(pulseRateRawBuf[prLast]);
    }
    //If the last value was 0, we shouuld have change be 0
    //Each measurement does not currently have its own counter 
    //so the display task only looks at an index across three
    else{
      change = 15;
    }
    if(change>=15||change<=-15){
        pulseRateRawBuf[prNext]=bpm;
    }

}
