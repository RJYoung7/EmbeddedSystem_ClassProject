/*
    FreeRTOS V9.0.0 - Copyright (C) 2016 Real Time Engineers Ltd.
    All rights reserved

    VISIT http://www.FreeRTOS.org TO ENSURE YOU ARE USING THE LATEST VERSION.

    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation >>>> AND MODIFIED BY <<<< the FreeRTOS exception.

    ***************************************************************************
    >>!   NOTE: The modification to the GPL is included to allow you to     !<<
    >>!   distribute a combined work that includes FreeRTOS without being   !<<
    >>!   obliged to provide the source code for proprietary components     !<<
    >>!   outside of the FreeRTOS kernel.                                   !<<
    ***************************************************************************

    FreeRTOS is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
    FOR A PARTICULAR PURPOSE.  Full license text is available on the following
    link: http://www.freertos.org/a00114.html

    ***************************************************************************
     *                                                                       *
     *    FreeRTOS provides completely free yet professionally developed,    *
     *    robust, strictly quality controlled, supported, and cross          *
     *    platform software that is more than just the market leader, it     *
     *    is the industry's de facto standard.                               *
     *                                                                       *
     *    Help yourself get started quickly while simultaneously helping     *
     *    to support the FreeRTOS project by purchasing a FreeRTOS           *
     *    tutorial book, reference manual, or both:                          *
     *    http://www.FreeRTOS.org/Documentation                              *
     *                                                                       *
    ***************************************************************************

    http://www.FreeRTOS.org/FAQHelp.html - Having a problem?  Start by reading
    the FAQ page "My application does not run, what could be wrong?".  Have you
    defined configASSERT()?

    http://www.FreeRTOS.org/support - In return for receiving this top quality
    embedded software for free we request you assist our global community by
    participating in the support forum.

    http://www.FreeRTOS.org/training - Investing in training allows your team to
    be as productive as possible as early as possible.  Now you can receive
    FreeRTOS training directly from Richard Barry, CEO of Real Time Engineers
    Ltd, and the world's leading authority on the world's leading RTOS.

    http://www.FreeRTOS.org/plus - A selection of FreeRTOS ecosystem products,
    including FreeRTOS+Trace - an indispensable productivity tool, a DOS
    compatible FAT file system, and our tiny thread aware UDP/IP stack.

    http://www.FreeRTOS.org/labs - Where new FreeRTOS products go to incubate.
    Come and try FreeRTOS+TCP, our new open source TCP/IP stack for FreeRTOS.

    http://www.OpenRTOS.com - Real Time Engineers ltd. license FreeRTOS to High
    Integrity Systems ltd. to sell under the OpenRTOS brand.  Low cost OpenRTOS
    licenses offer ticketed support, indemnification and commercial middleware.

    http://www.SafeRTOS.com - High Integrity Systems also provide a safety
    engineered and independently SIL3 certified version for use in safety and
    mission critical applications that require provable dependability.

    1 tab == 4 spaces!
*/


/*
 * Creates all the demo application tasks, then starts the scheduler.  The WEB
 * documentation provides more details of the standard demo application tasks.
 * In addition to the standard demo tasks, the following tasks and tests are
 * defined and/or created within this file:
 *
 * "Fast Interrupt Test" - A high frequency periodic interrupt is generated
 * using a free running timer to demonstrate the use of the
 * configKERNEL_INTERRUPT_PRIORITY configuration constant.  The interrupt
 * service routine measures the number of processor clocks that occur between
 * each interrupt - and in so doing measures the jitter in the interrupt timing.
 * The maximum measured jitter time is latched in the ulMaxJitter variable, and
 * displayed on the OLED display by the 'OLED' task as described below.  The
 * fast interrupt is configured and handled in the timertest.c source file.
 *
 * "OLED" task - the OLED task is a 'gatekeeper' task.  It is the only task that
 * is permitted to access the display directly.  Other tasks wishing to write a
 * message to the OLED send the message on a queue to the OLED task instead of
 * accessing the OLED themselves.  The OLED task just blocks on the queue waiting
 * for messages - waking and displaying the messages as they arrive.
 *
 * "Check" hook -  This only executes every five seconds from the tick hook.
 * Its main function is to check that all the standard demo tasks are still
 * operational.  Should any unexpected behaviour within a demo task be discovered
 * the tick hook will write an error to the OLED (via the OLED task).  If all the
 * demo tasks are executing with their expected behaviour then the check task
 * writes PASS to the OLED (again via the OLED task), as described above.
 *
 * "uIP" task -  This is the task that handles the uIP stack.  All TCP/IP
 * processing is performed in this task.
 */




/*************************************************************************
 * Please ensure to read http://www.freertos.org/portlm3sx965.html
 * which provides information on configuring and running this demo for the
 * various Luminary Micro EKs.
 *************************************************************************/

/* Set the following option to 1 to include the WEB server in the build.  By
default the WEB server is excluded to keep the compiled code size under the 32K
limit imposed by the KickStart version of the IAR compiler.  The graphics
libraries take up a lot of ROM space, hence including the graphics libraries
and the TCP/IP stack together cannot be accommodated with the 32K size limit. */
#define mainINCLUDE_WEB_SERVER		0


/* Standard includes. */
#include <stdio.h>
#include <string.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* Hardware library includes. */
//#include "hw_memmap.h"
//#include "hw_types.h"
#include "hw_sysctl.h"
//#include "inc/hw_ints.h"
//#include "sysctl.h"
//#include "gpio.h"
#include "grlib.h"
#include "rit128x96x4.h"
#include "osram128x64x4.h"
#include "formike128x128x16.h"
//#include "driverlib/interrupt.h"
//#include "driverlib/timer.h"
//#include "driverlib/systick.h"
//#include "driverlib/pwm.h"
#include "adc.h"

/* Demo app includes. */
//#include "BlockQ.h"
//#include "death.h"
//#include "integer.h"
//#include "blocktim.h"
//#include "flash.h"
#include "partest.h"
//#include "semtest.h"
//#include "PollQ.h"
#include "lcd_message.h"
#include "bitmap.h"
//#include "GenQTest.h"
//#include "QPeek.h"
//#include "recmutex.h"
//#include "IntQueue.h"
//#include "QueueSet.h"
//#include "EventGroupsDemo.h"

/* Task includes */
//#include "measureTask.h"
//#include "computeTask.h"
//#include "displayTask.h"
//#include "serialComTask.h"
//#include "warningAlarm.h"
//#include "Flags.h"
//#include "systemTimeBase.h"

/* Datastruct includes */
//#include "dataPtrs.h"
//#include "dataStructs.c"

/* Project 3 */
#include "inc/hw_types.h"
#include "computeTask.h"
#include "dataPtrs.h"
#include "displayTask.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"
#include "driverlib/pwm.h"
#include "drivers/rit128x96x4.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/uart.h"
#include "dataStructs.c"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
//#include "inc/lm3s8962.h"
#include "measureTask.h"
#include "serialComTask.h"
#include "systemTimeBase.h"
#include "warningAlarm.h"
#include "Flags.h"

#include <time.h>

#define CLOCK_RATE      300


/*-----------------------------------------------------------*/

/* The time between cycles of the 'check' functionality (defined within the
tick hook. */
#define mainCHECK_DELAY						( ( TickType_t ) 5000 / portTICK_PERIOD_MS )

/* Size of the stack allocated to the uIP task. */
#define mainBASIC_WEB_STACK_SIZE            ( configMINIMAL_STACK_SIZE * 3 )

/* The OLED task uses the sprintf function so requires a little more stack too. */
#define mainOLED_TASK_STACK_SIZE			( configMINIMAL_STACK_SIZE + 50 )

/* Task priorities. */
#define mainQUEUE_POLL_PRIORITY				( tskIDLE_PRIORITY + 2 )
#define mainCHECK_TASK_PRIORITY				( tskIDLE_PRIORITY + 3 )
#define mainSEM_TEST_PRIORITY				( tskIDLE_PRIORITY + 1 )
#define mainBLOCK_Q_PRIORITY				( tskIDLE_PRIORITY + 2 )
#define mainCREATOR_TASK_PRIORITY           ( tskIDLE_PRIORITY + 3 )
#define mainINTEGER_TASK_PRIORITY           ( tskIDLE_PRIORITY )
#define mainGEN_QUEUE_TASK_PRIORITY			( tskIDLE_PRIORITY )

/* The maximum number of message that can be waiting for display at any one
time. */
#define mainOLED_QUEUE_SIZE					( 3 )

/* Dimensions the buffer into which the jitter time is written. */
#define mainMAX_MSG_LEN						25

/* The period of the system clock in nano seconds.  This is used to calculate
the jitter time in nano seconds. */
#define mainNS_PER_CLOCK					( ( unsigned long ) ( ( 1.0 / ( double ) configCPU_CLOCK_HZ ) * 1000000000.0 ) )

/* Constants used when writing strings to the display. */
#define mainCHARACTER_HEIGHT				( 9 )
#define mainMAX_ROWS_128					( mainCHARACTER_HEIGHT * 14 )
#define mainMAX_ROWS_96						( mainCHARACTER_HEIGHT * 10 )
#define mainMAX_ROWS_64						( mainCHARACTER_HEIGHT * 7 )
#define mainFULL_SCALE						( 15 )
#define ulSSI_FREQUENCY						( 3500000UL )

/*-----------------------------------------------------------*/

// Global counters
unsigned volatile int globalCounter = 0;
unsigned int auralCounter = 0;
unsigned int pulseFreq=4;
unsigned int pulseCount=0;
unsigned long g_ulFlagPR=0;

//*****************************************************************************
//
// Flags that contain the current value of the interrupt indicator as displayed
// on the OLED display.
//
//*****************************************************************************
unsigned long g_ulFlags;
unsigned long auralFlag;
unsigned long ackFlag = 0;
unsigned long computeFlag;
unsigned long serialFlag;
TaskHandle_t xComputeHandle;
TaskHandle_t xDisplayHandle;
TaskHandle_t xTempHandle;

//*****************************************************************************
//
// A set of flags used to track the state of the application.
//
//*****************************************************************************

//extern unsigned long g_ulFlags;
#define FLAG_CLOCK_TICK         0           // A timer interrupt has occurred
#define FLAG_CLOCK_COUNT_LOW    1           // The low bit of the clock count
#define FLAG_CLOCK_COUNT_HIGH   2           // The high bit of the clock count
#define FLAG_UPDATE             3           // The display should be updated
#define FLAG_BUTTON             4           // Debounced state of the button
#define FLAG_DEBOUNCE_LOW       5           // Low bit of the debounce clock
#define FLAG_DEBOUNCE_HIGH      6           // High bit of the debounce clock
#define FLAG_BUTTON_PRESS       7           // The button was just pressed
#define FLAG_ENET_RXPKT         8           // An Ethernet Packet received
#define FLAG_ENET_TXPKT         9           // An Ethernet Packet transmitted

//*****************************************************************************
//
// The speed of the processor.
//
//*****************************************************************************

unsigned long g_ulSystemClock;

//*****************************************************************************
//
// The debounced state of the five push buttons.  The bit positions correspond
// to:
//
//     0 - Up
//     1 - Down
//     2 - Left
//     3 - Right
//     4 - Select
//
//*****************************************************************************

unsigned char g_ucSwitches = 0x1f;

//*****************************************************************************
//
// The vertical counter used to debounce the push buttons.  The bit positions
// are the same as g_ucSwitches.
//
//*****************************************************************************

static unsigned char g_ucSwitchClockA = 0;
static unsigned char g_ucSwitchClockB = 0;

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, unsigned long ulLine)
{
}
#endif

//  Declare the globals
INIT_MEASUREMENT2(m2);
INIT_DISPLAY2(d2);
INIT_STATUS(s1);
INIT_ALARMS(a1);
INIT_WARNING(w1);
INIT_SCHEDULER(c1);
INIT_KEYPAD(k1);

//Connect pointer structs to data
measureData2 mPtrs2 = 
{     
  m2.temperatureRawBuf,
  m2.bloodPressRawBuf,
  m2.pulseRateRawBuf,
  &m2.countCalls,
  &m2.sysComplete,
  &m2.diaComplete,
  &m2.tempDirection,
  &g_ulFlagPR,
  &m2.cuffPressRaw	
};

computeData2 cPtrs2=
{
  m2.temperatureRawBuf,
  m2.bloodPressRawBuf,
  m2.pulseRateRawBuf,
  d2.tempCorrectedBuf,
  d2.bloodPressCorrectedBuf,
  d2.pulseRateCorrectedBuf,
  &k1.measurementSelection,
  &m2.countCalls
};

displayData2 dPtrs2=
{
  d2.tempCorrectedBuf,
  d2.bloodPressCorrectedBuf,
  d2.pulseRateCorrectedBuf,
  &s1.batteryState,
  &m2.countCalls,
  &k1.mode,
  &a1.tempOutOfRange,
  &a1.bpOutOfRange,
  &a1.pulseOutOfRange
  
};

warningAlarmData2 wPtrs2=
{
  m2.temperatureRawBuf,
  m2.bloodPressRawBuf,
  m2.pulseRateRawBuf,
  &s1.batteryState,
  &a1.bpOutOfRange,
  &a1.tempOutOfRange,
  &a1.pulseOutOfRange,
  &w1.bpHigh,
  &w1.tempHigh,
  &w1.pulseLow,
  &w1.led,
  &m2.countCalls,
  &w1.previousCount,
  &w1.pulseFlash,
  &w1.tempFlash,
  &w1.bpFlash,
  &w1.auralCount
};

keypadData kPtrs=
{
  &k1.mode,
  &k1.measurementSelection,
  &k1.scroll,
  &k1.selectChoice,
  &k1.alarmAcknowledge,
  &m2.cuffPressRaw

};

statusData sPtrs=
{  
  &s1.batteryState
};

schedulerData schedPtrs=
{
  &c1.globalCounter
};

communicationsData comPtrs={
  d2.tempCorrectedBuf,
  d2.bloodPressCorrectedBuf,
  d2.pulseRateCorrectedBuf,
  &s1.batteryState,
  &m2.countCalls
};

//Declare the prototypes for the tasks
void compute(void* data);
void measure(void* data);
void stat(void* data);
void alarm(void* data);
void disp(void* data);
void schedule(void* data);
void keypadfunction(void* data);
void startup();

/*
 * The task that handles the uIP stack.  All TCP/IP processing is performed in
 * this task.
 */
extern void vuIP_Task( void *pvParameters );

/*
 * The display is written two by more than one task so is controlled by a
 * 'gatekeeper' task.  This is the only task that is actually permitted to
 * access the display directly.  Other tasks wanting to display a message send
 * the message to the gatekeeper.
 */
static void vOLEDTask( void *pvParameters );

/*
 * Configure the hardware for the demo.
 */
static void prvSetupHardware( void );

/*
 * Configures the high frequency timers - those used to measure the timing
 * jitter while the real time kernel is executing.
 */
extern void vSetupHighFrequencyTimer( void );

/*
 * Hook functions that can get called by the kernel.
 */
void vApplicationStackOverflowHook( TaskHandle_t *pxTask, signed char *pcTaskName );
void vApplicationTickHook( void );


/*-----------------------------------------------------------*/

/* The queue used to send messages to the OLED task. */
QueueHandle_t xOLEDQueue;

/* The welcome text. */
const char * const pcWelcomeMessage = "   www.FreeRTOS.org";

/* Task Prototypes */
void vTask1(void *vParameters);

/*-----------------------------------------------------------*/

//*****************************************************************************
//
// Handles the SysTick timeout interrupt.
//
//*****************************************************************************

void
SysTickIntHandler(void)
{
  unsigned long ulData, ulDelta;

  // Indicate that a timer interrupt has occurred.
  HWREGBITW(&g_ulFlags, FLAG_CLOCK_TICK) = 1;
  
  portDISABLE_INTERRUPTS();
	{
		/* Increment the RTOS tick. */
		if( xTaskIncrementTick() != pdFALSE )
		{
			/* A context switch is required.  Context switching is performed in
			the PendSV interrupt.  Pend the PendSV interrupt. */
			portNVIC_INT_CTRL_REG = portNVIC_PENDSVSET_BIT;
		}
	}
	portENABLE_INTERRUPTS();
        
  // only check buttons if there is not a button pressed
  if(!HWREGBITW(&g_ulFlags, FLAG_BUTTON_PRESS)){
    // Read the state of the push buttons.
    ulData = (GPIOPinRead(GPIO_PORTE_BASE, (GPIO_PIN_0 | GPIO_PIN_1 |
                                            GPIO_PIN_2 | GPIO_PIN_3)) |
              (GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_1) << 3));

    // Determine the switches that are at a different state than the debounced state.
    //debug line to imitate up click
    ulDelta = ulData ^ g_ucSwitches;

    // Increment the clocks by one.
    // Exclusive or of clock B If a bit is different in A and B then 1 if the bits have the same value = 0
    g_ucSwitchClockA ^= g_ucSwitchClockB;
    
    // Compliment of clock B. This changes 1 to 0 and 0 to 1 bitwise
    g_ucSwitchClockB = ~g_ucSwitchClockB;

    // Reset the clocks corresponding to switches that have not changed state.
    g_ucSwitchClockA &= ulDelta;
    g_ucSwitchClockB &= ulDelta;

    // Get the new debounced switch state.
    g_ucSwitches &= g_ucSwitchClockA | g_ucSwitchClockB;
    g_ucSwitches |= (~(g_ucSwitchClockA | g_ucSwitchClockB)) & ulData;

    // Determine the switches that just changed debounced state.
    ulDelta ^= (g_ucSwitchClockA | g_ucSwitchClockB);

    // See if the select button was  pressed during an alarm.
    if(g_ucSwitches==15 && auralFlag==1)
    {
        // Set a flag to indicate that the select button was just pressed.
        PWMGenDisable(PWM_BASE, PWM_GEN_0);
        auralFlag = 0;
        auralCounter = globalCounter;
        ackFlag = 1;
    }
    // See if any switches just changed debounced state.
    if(ulDelta && (g_ucSwitches != 0x1F))
    {
        // You can watch the variable for ulDelta
        // Up = 1 Right = 8 down =2 left =4  select = 16 Bit values
        //printf("A button was pressed %d \n", ulDelta);
        //printf("SwitchesState %d \n", g_ucSwitches);
        HWREGBITW(&g_ulFlags, FLAG_BUTTON_PRESS) = 1;
        
    }
  }
}

//*****************************************************************************
//
// The interrupt handler for the first timer interrupt.
//
//*****************************************************************************
void
Timer0IntHandler(void)
{
    // Clear the timer interrupt.
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    // Update the global counter.
    IntMasterDisable();
    increment();
    IntMasterEnable();
}


//void
//GPIOFIntHandler(void)
//{
//    // Clear the timer interrupt.
//    GPIOPinIntClear(GPIO_PORTF_BASE, GPIO_PIN_1);
//    
//    static unsigned char ucLocalTickCount = 0;
//    BaseType_t xHigherPriorityTaskWoken;
//
//    /* Is it time for vATask() to run? */
//    xHigherPriorityTaskWoken = pdFALSE;
//    ucLocalTickCount++;
//
//        /* Unblock the task by releasing the semaphore. */
//    if( ucLocalTickCount >= TICKS_TO_WAIT)
//    {
//        xSemaphoreGiveFromISR( xSemaphore, &xHigherPriorityTaskWoken );
//        ucLocalTickCount = 0;
//    }
//
//    /* If xHigherPriorityTaskWoken was set to true you
//    we should yield.  The actual macro used here is
//    port specific. */
//    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
//
//
//    // Update the global counter.
//    //IntMasterDisable();
//    //GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, 0x01);
//    
//    //IntMasterEnable();
//}


//*****************************************************************************
//
// The interrupt handler for the pulse rate transducer interrupt.
//
//*****************************************************************************
void
Timer1IntHandler(void)
{
    // Clear the timer interrupt.
    TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
    
    pulseCount++;
        
    if(pulseCount>20){
      pulseFreq=3;
    }
    else if(pulseCount >40){
      pulseFreq = 2;
    }
    else if(pulseCount >60){
      pulseFreq=1;
    }
    else if (pulseCount>80){
      pulseFreq =4;
      pulseCount =0;
    }
    
    TimerLoadSet(TIMER1_BASE, TIMER_A, (SysCtlClockGet()/pulseFreq)/2-1);

    // Update PR Flag
    if(g_ulFlagPR==0){
      g_ulFlagPR=1;
    }
    else
      g_ulFlagPR=0;
}


/*************************************************************************
 * Please ensure to read http://www.freertos.org/portlm3sx965.html
 * which provides information on configuring and running this demo for the
 * various Luminary Micro EKs.
 *************************************************************************/
int main( void )
{      
    prvSetupHardware();

    /* Create the queue used by the OLED task.  Messages for display on the OLED
    are received via this queue. */
    xOLEDQueue = xQueueCreate( mainOLED_QUEUE_SIZE, sizeof( xOLEDMessage ) );
    
    // Create tasks
    xTaskCreate(measure, "Measure Task", 2048, (void*)&mPtrs2, 3, NULL);
    xTaskCreate(alarm, "Warning Task", 500, (void*)&wPtrs2, 4, NULL);
    xTaskCreate(stat, "Status Task", 100, (void*)&sPtrs, 3, NULL);
    xTaskCreate(compute, "Compute Task", 100, (void*)&cPtrs2, 2, &xComputeHandle);
    xTaskCreate(disp, "Display Task", 1024, (void*)&dPtrs2, 2, &xDisplayHandle);
    xTaskCreate(keypadfunction, "Keypad Task", 500, (void*)&kPtrs, 1, NULL);
    
    /* Exclude some tasks if using the kickstart version to ensure we stay within
    the 32K code size limit. */
    #if mainINCLUDE_WEB_SERVER != 1
    {
            /* Create the uIP task if running on a processor that includes a MAC and
            PHY. */
            if( SysCtlPeripheralPresent( SYSCTL_PERIPH_ETH ) )
            {
                    xTaskCreate( vuIP_Task, "uIP", mainBASIC_WEB_STACK_SIZE, NULL, mainCHECK_TASK_PRIORITY - 1, NULL );
            }
    }
    #endif

    /* Start the tasks defined within this file/specific to this demo. */
    xTaskCreate( vOLEDTask, "OLED", mainOLED_TASK_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL );
    
    /* Configure the high frequency interrupt used to measure the interrupt
    jitter time. */
    //vSetupHighFrequencyTimer();

    /* Start the scheduler. */
    vTaskStartScheduler();

    /* Will only get here if there was insufficient memory to create the idle
    task. */
	return 0;
}
/*-----------------------------------------------------------*/
  
void prvSetupHardware( void )
{
    // Variables used for configurations
  unsigned long ulPeriod;
  unsigned long ulPeriodPR;
  /* If running on Rev A2 silicon, turn the LDO voltage up to 2.75V.  This is
  a workaround to allow the PLL to operate reliably. */
  if( DEVICE_IS_REVA2 )
  {
      SysCtlLDOSet( SYSCTL_LDO_2_75V );
  }

  /* Set the clocking to run from the PLL at 50 MHz */
  SysCtlClockSet( SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_8MHZ );

  /* 	Enable Port F for Ethernet LEDs
          LED0        Bit 3   Output
          LED1        Bit 2   Output 
  */
  
  g_ulSystemClock = SysCtlClockGet();
  
  // Enable the peripherals used by this example.
  SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1); 
  SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
  
  // Configure the GPIO used to output the state of the led
  GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_0);
  GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2);//GPIO_PF2_LED1
  GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_3);

  //**INITIALIZE BUTTONS**//
  //Configure the GPIOs used to read the state of the on-board push buttons.
  GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
  GPIOPadConfigSet(GPIO_PORTE_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3,
                   GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
  GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_1);
  GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
  GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0);
  
  // Configure SysTick to periodically interrupt.
  SysTickPeriodSet(g_ulSystemClock / CLOCK_RATE);
  SysTickIntEnable();
  SysTickEnable();
  
  /* ADC BEGIN*/
  // The ADC0 peripheral must be enabled for use.
  SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);

  /* Enable sample sequence 3 with a processor signal trigger.  Sequence 3
   will do a single sample when the processor sends a singal to start the
   conversion.  Each ADC module has 4 programmable sequences, sequence 0
   to sequence 3.  This example is arbitrarily using sequence 3. */
  ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);

  /* Configure step 0 on sequence 3.  Sample the temperature sensor
  (ADC_CTL_TS) and configure the interrupt flag (ADC_CTL_IE) to be set
   when the sample is done.  Tell the ADC logic that this is the last
   conversion on sequence 3 (ADC_CTL_END).  Sequence 3 has only one
   programmable step.  Sequence 1 and 2 have 4 steps, and sequence 0 has
   8 programmable steps.  Since we are only doing a single conversion using
   sequence 3 we will only configure step 0.  For more information on the
   ADC sequences and steps, reference the datasheet.*/
  ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_TS | ADC_CTL_IE |
                           ADC_CTL_END);

  // Since sample sequence 3 is now configured, it must be enabled.
  ADCSequenceEnable(ADC0_BASE, 3);

  /* Clear the interrupt status flag.  This is done to make sure the
   interrupt flag is cleared before we sample.*/
  ADCIntClear(ADC0_BASE, 3);
  
  /*ADC END*/
  
  //**INITIALIZE UART**//
  // Configure the GPIO for the UART
  GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
     
  // Set the configuration of the UART
  UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 460800,
                        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                         UART_CONFIG_PAR_NONE));

  
  
  //**INITIALIZE TIMER INTERRUPT**//
  // Configure the 32-bit periodic timer.
  TimerConfigure(TIMER0_BASE, TIMER_CFG_32_BIT_PER);
  TimerConfigure(TIMER1_BASE, TIMER_CFG_32_BIT_PER);
  
  ulPeriodPR =(SysCtlClockGet()/400)/200 ;
  
  TimerLoadSet(TIMER0_BASE, TIMER_A, SysCtlClockGet()/1000);
  TimerLoadSet(TIMER1_BASE, TIMER_A, ulPeriodPR-1);

  // Setup the interrupt for the timer timeout.
  IntEnable(INT_TIMER0A);
  IntEnable(INT_TIMER1A);

  TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
  TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);

  // Enable the timer.
  TimerEnable(TIMER0_BASE, TIMER_A);
  TimerEnable(TIMER1_BASE, TIMER_A);
  
  //**INITIAL SOUND WARNING**//
  // Set GPIO G1 as PWM pin.  They are used to output the PWM1 signal.
  GPIOPinTypePWM(GPIO_PORTG_BASE, GPIO_PIN_1);
  
  // Compute the PWM period based on the system clock.
  ulPeriod = SysCtlClockGet() / 440;
  
  // Set the PWM period to 440 (A) Hz.
  PWMGenConfigure(PWM_BASE, PWM_GEN_0,
                    PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
  PWMGenPeriodSet(PWM_BASE, PWM_GEN_0, ulPeriod);

  // PWM1 to a duty cycle of 75%.
  PWMPulseWidthSet(PWM_BASE, PWM_OUT_1, ulPeriod * 3 / 4);

  // Enable the PWM1 output signal.
  PWMOutputState(PWM_BASE,PWM_OUT_1_BIT, true);
  
  // Enable processor interrupts.
  IntMasterEnable();
  
  // Turn on LED to indicate normal state
  enableVisibleAnnunciation();

  //vParTestInitialise();
}
/*-----------------------------------------------------------*/

void vApplicationTickHook( void )
{

  static xOLEDMessage xMessage = { "PASS" };
  static unsigned long ulTicksSinceLastDisplay = 0;
  portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

  /* Called from every tick interrupt.  Have enough ticks passed to make it
  time to perform our health status check again? */

  ulTicksSinceLastDisplay++;
  if( ulTicksSinceLastDisplay >= mainCHECK_DELAY )
  {
          ulTicksSinceLastDisplay = 0;

  }
}
/*-----------------------------------------------------------*/

void vOLEDTask( void *pvParameters )
{
  xOLEDMessage xMessage;
  unsigned long ulY, ulMaxY;
  static char cMessage[ mainMAX_MSG_LEN ];
  extern volatile unsigned long ulMaxJitter;
  const unsigned char *pucImage;

  /* Functions to access the OLED.  The one used depends on the dev kit
  being used. */
  void ( *vOLEDInit )( unsigned long ) = NULL;
  void ( *vOLEDStringDraw )( const char *, unsigned long, unsigned long, unsigned char ) = NULL;
  void ( *vOLEDImageDraw )( const unsigned char *, unsigned long, unsigned long, unsigned long, unsigned long ) = NULL;
  void ( *vOLEDClear )( void ) = NULL;

  /* Map the OLED access functions to the driver functions that are appropriate
  for the evaluation kit being used. */
  vOLEDInit = RIT128x96x4Init;
  vOLEDStringDraw = RIT128x96x4StringDraw;
  vOLEDImageDraw = RIT128x96x4ImageDraw;
  vOLEDClear = RIT128x96x4Clear;
  ulMaxY = mainMAX_ROWS_96;
  pucImage = pucBasicBitmap;


  ulY = ulMaxY;

  /* Initialise the OLED and display a startup message. */
  vOLEDInit( ulSSI_FREQUENCY );
  //vOLEDStringDraw( "POWERED BY FreeRTOS", 0, 0, mainFULL_SCALE );
  //vOLEDImageDraw( pucImage, 0, mainCHARACTER_HEIGHT + 1, bmpBITMAP_WIDTH, bmpBITMAP_HEIGHT );

  for( ;; )
  {
          /* Wait for a message to arrive that requires displaying. */
          xQueueReceive( xOLEDQueue, &xMessage, portMAX_DELAY );

          /* Write the message on the next available row. */
          ulY += mainCHARACTER_HEIGHT;
          if( ulY >= ulMaxY )
          {
                  ulY = mainCHARACTER_HEIGHT;
                  vOLEDClear();
                  vOLEDStringDraw( pcWelcomeMessage, 0, 0, mainFULL_SCALE );
          }

          /* Display the message along with the maximum jitter time from the
          high priority time test. */
          sprintf( cMessage, "%s [%uns]", xMessage.pcMessage, ulMaxJitter * mainNS_PER_CLOCK );
          vOLEDStringDraw( cMessage, 0, ulY, mainFULL_SCALE );
	}
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( TaskHandle_t *pxTask, signed char *pcTaskName )
{
	( void ) pxTask;
	( void ) pcTaskName;

	for( ;; );
}
/*-----------------------------------------------------------*/

void vAssertCalled( const char *pcFile, unsigned long ulLine )
{
volatile unsigned long ulSetTo1InDebuggerToExit = 0;

	taskENTER_CRITICAL();
	{
		while( ulSetTo1InDebuggerToExit == 0 )
		{
			/* Nothing do do here.  Set the loop variable to a non zero value in
			the debugger to step out of this function to the point that caused
			the assertion. */
			( void ) pcFile;
			( void ) ulLine;
		}
	}
	taskEXIT_CRITICAL();
}
