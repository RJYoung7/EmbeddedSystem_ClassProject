#include "ekgProcessTask.h"
#include "dataStructs.c"
#include "dataPtrs.c"
#include "systemTimeBase.h"
#include "FreeRTOS.h"
#include "task.h"
#include "math.h"
#include "optfft.h"

extern int optfft();

 void ekgProcess(void *data) {
  EKGData * EKGDataPtr = (EKGData*) data;
  signed int* EKGRawBuf = (*EKGDataPtr).EKGRawBufPtr;
  for( ;; )
  {
    // check to see if the ekg has been captured via a flag
    // Define the imaginary buffer
    signed int imgBuf[256];
    // initialize to zeros
    for(int i = 0; i<256; i++){
    imgBuf[i] = 0;
    }
    
    //use brents optimized fft formula
    //hard coded a single index for teesting. Will need to add an index staticvariable
    
    (*EKGDataPtr).EKGFreqBufPtr[0] = optfft(EKGRawBuf,imgBuf);
    
      vTaskDelay(5000);
  }
  //return;

 }
	


 