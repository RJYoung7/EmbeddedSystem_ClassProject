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
    
    (*EKGDataPtr).EKGFreqBufPtr[ekgCounter] = optfft(EKGRawBuf,imgBuf);
    
    // EKG index increment
    ekgCounter = (ekgCounter + 1 )%8;  
    vTaskSuspend( NULL );
    //vTaskDelay(5000);
  }
  //return;

 }
	


 