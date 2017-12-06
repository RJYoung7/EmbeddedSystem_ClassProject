#include "ekgProcessTask.h"
#include "dataStructs.c"
#include "dataPtrs.c"
#include "systemTimeBase.h"
#include "FreeRTOS.h"
#include "Flags.h"
#include "task.h"
#include "math.h"
#include "optfft.h"

extern int optfft();

 void ekgProcess(void *data) {
  EKGData * EKGDataPtr = (EKGData*) data;
  signed int* EKGRawBuf = (*EKGDataPtr).EKGRawBufPtr;
  for( ;; )
  {
    
    
    // Define the imaginary buffer
    signed int imgBuf[256];
    // initialize to zeros
    for(int i = 0; i<256; i++){
    imgBuf[i] = 0;
    }
    
    //use brents optimized fft formula
    //hard coded a single index for teesting. Will need to add an index staticvariable
    // Brents algorithm returns the index of the Maxima of the sine wave
    signed int index = optfft(EKGRawBuf,imgBuf);
    // Using this index we can find the frequency in the input array
    (*EKGDataPtr).EKGFreqBufPtr[ekgCounter] = EKGRawBuf[index];
    // EKG index increment
    ekgCounter = (ekgCounter + 1 )%16;  
    vTaskSuspend( NULL );
    //vTaskDelay(5000);
  }
  //return;

 }
	


 