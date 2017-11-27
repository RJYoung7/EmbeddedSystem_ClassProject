#include "ekgCaptureTask.h"
#include "dataStructs.c"
#include "dataPtrs.c"
#include "systemTimeBase.h"
#include "FreeRTOS.h"
#include "Flags.h"
#include "task.h"
#include "math.h"

 void ekgCapture(void *data) {
  EKGData * EKGDataPtr = (EKGData*) data;
  signed int* EKGRawBuf = (*EKGDataPtr).EKGRawBufPtr;
  for( ;; )
  {
    //Generate the sine wave
    float pi = 3.1417;
    float w = 2 * pi * 1000;
    float t =0;
    for(int i = 0; i<256; i++)
    {
      // store wave value at a given time t in the raw buffer
      // frequency is w or 2000Pi
      EKGRawBuf[i] = (int) (30.0 * sin(w*t));
      // increment t
      // May need to change t to fulfill the following req:  equally spaced
      //samples with inter sample temporal spacing to model a sampling rate at two and a half to
      //three times the maximum specified frequency.
      t=t+.000125;
    }
      // signal Process
      vTaskResume(xEKGHandle);
      // delay to allow other tasks to run
      vTaskDelay(5000);
  }
  //return;

 }
	


 