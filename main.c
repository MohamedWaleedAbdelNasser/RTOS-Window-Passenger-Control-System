#include <stdint.h>
#include <string.h>
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>
#include "hw_ints.h"
#include "hw_memmap.h"
#include "hw_sysctl.h"
#include "TM4C123GH6PM.h"
#include "Dio.h"
#include "types.h"
#define Jam_mode    0
#define Forward     1
#define Backward    2
#define OFF         3



/*initializations*/
void PORTF_INIT(void);
void PORTB_INIT(void);
void PORTA_INIT(void);
void PORTD_INIT(void);

void ManualControlTask(void* parameters);
void MotorControlTask( void *pvParameters );
void JammerTask(void *pvParameters);
void Motor_Forward(void);
void Motor_Backward(void);
void Motor_Off(void);
void delayMs(int n);

/*Semaphores*/
SemaphoreHandle_t xJamAutoSemaphore;
SemaphoreHandle_t xJamPressedSemaphore;

// Define task handles
xTaskHandle switchTaskHandle;
xTaskHandle functionTaskHandle;
xTaskHandle passenger;

// Define Queue handles
xQueueHandle xMotorQueue;

//Global Variables
static int x=0;
int mode=0;

/*------------------Functions----------*/
void delayMs(int n){
  int i,j;
  for(i=0;i<n;i++)
  {
    for(j=0;j<3180;j++)
    {}
  }
}

void Motor_Forward(void){
			DIO_WritePin(&GPIO_PORTA_DATA_R,2,1);
			DIO_WritePin(&GPIO_PORTA_DATA_R,3,0);
}
void Motor_Backward(void){
			DIO_WritePin(&GPIO_PORTA_DATA_R,2,0);
			DIO_WritePin(&GPIO_PORTA_DATA_R,3,1);
}
void Motor_Off(void){
			DIO_WritePin(&GPIO_PORTA_DATA_R,2,1);
			DIO_WritePin(&GPIO_PORTA_DATA_R,3,1);
}


// Polling task function checking for all inputs
void ManualControlTask(void* parameters) {
	portBASE_TYPE xStatus;
    while (1) {
				if((GPIO_PORTB_DATA_R & 0x01)==0x01){
					xSemaphoreGive(xJamAutoSemaphore);
					xStatus=xQueueSendToBack(xMotorQueue,&(uint32_t){Forward},0);
				}
				if((GPIO_PORTB_DATA_R & 0x02)==0x02){
					xStatus=xQueueSendToBack(xMotorQueue,&(uint32_t){Backward},0);
				}
        while ((GPIO_PORTB_DATA_R & 0x04)==0x04) {
						xStatus=xQueueSendToBack(xMotorQueue,&(uint32_t){Forward},0);
					if((GPIO_PORTB_DATA_R & 0x04)!=0x04){
							xStatus=xQueueSendToBack(xMotorQueue,&(uint32_t){OFF},0);
							break;
					}
				}
				while ((GPIO_PORTB_DATA_R & 0x08)==0x08){
						xStatus=xQueueSendToBack(xMotorQueue,&(uint32_t){Backward},0);
					if((GPIO_PORTB_DATA_R & 0x08)!=0x08){
							xStatus=xQueueSendToBack(xMotorQueue,&(uint32_t){OFF},0);
							break;
					}
				}
				if(((GPIO_PORTB_DATA_R & 0x10)==0x10) && ((GPIO_PORTA_DATA_R & 0x40) != 0x40)){
					xSemaphoreGive(xJamAutoSemaphore);
					xStatus=xQueueSendToBack(xMotorQueue,&(uint32_t){Forward},0);
				}
				if(((GPIO_PORTB_DATA_R & 0x20)==0x20) && ((GPIO_PORTA_DATA_R & 0x40) != 0x40)){
					xStatus=xQueueSendToBack(xMotorQueue,&(uint32_t){Backward},0);
				}
				while (((GPIO_PORTB_DATA_R & 0x40) == 0x40) && ((GPIO_PORTA_DATA_R & 0x40) != 0x40))
				{ 
						xStatus=xQueueSendToBack(xMotorQueue,&(uint32_t){Forward},0);
					if((GPIO_PORTB_DATA_R & 0x40)!=0x40){
							xStatus=xQueueSendToBack(xMotorQueue,&(uint32_t){OFF},0);
							break;
					}
				}
				while (((GPIO_PORTB_DATA_R & 0x80)==0x80) && ((GPIO_PORTA_DATA_R & 0x40) != 0x40))
				{ 
					xStatus=xQueueSendToBack(xMotorQueue,&(uint32_t){Backward},0);
					if((GPIO_PORTB_DATA_R & 0x80)!=0x80){
						xStatus=xQueueSendToBack(xMotorQueue,&(uint32_t){OFF},0);
						break;
					}
				}
        vTaskDelay(pdMS_TO_TICKS(10)); // debounce switch
    }
}


/*-------------------Jammer--------*/
void JammerTask(void *pvParameters){
	long message=(long)pvParameters;
	portBASE_TYPE xStatus;
	xSemaphoreTake(xJamAutoSemaphore,0);
	for(;;)
	{
			xSemaphoreTake(xJamAutoSemaphore,portMAX_DELAY);
		  xSemaphoreTake(xJamPressedSemaphore,portMAX_DELAY);
			xStatus=xQueueSendToBack(xMotorQueue,&(uint32_t){0},0);
	}
}
/*-----------MotorController-------*/
void MotorControlTask(void *pvParameters){
	uint32_t message;
	for(;;)
{
	xQueueReceive(xMotorQueue,&message,portMAX_DELAY);
	
	switch(message){
		case Jam_mode :
			Motor_Off();
			delayMs(1000);
			Motor_Backward();   	//Jamm Mode Motor stops then move backwards
			delayMs(1000);
			Motor_Off();
			break;
		case Forward:
			Motor_Forward();    //Motor forward
			break;
		case Backward :
			Motor_Backward();   //Motor backward
			break;		
		case OFF:
			Motor_Off(); 				//Motor Stops
			break;
		}
	}
}

                         
int main( void )
{
	PORTA_INIT();
  PORTF_INIT();
	PORTB_INIT();
	PORTD_INIT();
	
	xMotorQueue=xQueueCreate(20,sizeof(uint32_t));	
	xJamAutoSemaphore = xSemaphoreCreateBinary();
	xJamPressedSemaphore = xSemaphoreCreateBinary();
	__ASM("CPSIE i");
	
	if( (xJamAutoSemaphore && xJamPressedSemaphore && xMotorQueue) != NULL )
		{
			xTaskCreate(ManualControlTask, "Switch Task", configMINIMAL_STACK_SIZE, NULL, 1, switchTaskHandle);
			xTaskCreate( MotorControlTask, "Motor",configMINIMAL_STACK_SIZE, NULL, 2, NULL );
			xTaskCreate (JammerTask, "Jammer", configMINIMAL_STACK_SIZE,NULL,3,NULL);
			
			
			/* Start the scheduler so the created tasks start executing. */
			vTaskStartScheduler();
		}

    for( ;; );
}

/*-----------------------------Initializations-------------------------------------------*/
void PORTF_INIT(void){ 
  SYSCTL->RCGCGPIO |= 0x00000020;    // 1) F clock
  GPIOF->LOCK = 0x4C4F434B;  				 // 2) unlock PortF PF0  
  GPIOF->CR = 0x1F;          				 // allow changes to PF4-0       
  GPIOF->AMSEL= 0x00;       				 // 3) disable analog function
  GPIOF->PCTL = 0x00000000;  				 // 4) GPIO clear bit PCTL  
  GPIOF->DIR = 0x0E;         				 // 5) PF4,PF0 input, PF3,PF2,PF1 output   
  GPIOF->AFSEL = 0x00;      				 // 6) no alternate function
  GPIOF->PUR = 0x11;       				   // enable pullup resistors on PF4,PF0       
  GPIOF->DEN = 0x1F;       				   // 7) enable digital pins PF4-PF0
	GPIOF->DATA = 0x00;
	GPIOF->ICR = 0x11;     // Clear any Previous Interrupt 
	GPIOF->IM |=0x11;      // Unmask the interrupts for PF0 and PF4
	GPIOF->IS |= 0x11;     // Make bits PF0 and PF4 level sensitive
	GPIOF->IEV &= ~0x11;   // Sense on Low Level
	NVIC_EN0_R |= 0x40000000;
  NVIC_PRI7_R |= 0xe0000;  // Enable the Interrupt for PortF in NVIC
}

void PORTB_INIT(void){
	SYSCTL->RCGCGPIO |= 0x00000002; 	// initialize clock                           // looping until clock is initiallized
	GPIOB->LOCK = 0x4C4F434B;
	GPIOB->AMSEL= 0x00;       				 // 3) disable analog function
  GPIOB->PCTL = 0x00000000;  				 // 4) GPIO clear bit PCTL 
	GPIOB->CR |= 0x000000ff;                                    // unlocking commit register for switch 1 & switch 2 
  GPIOB->DIR |= 0x00000000;                            // detrmining the output pins                                   
  GPIOB->AFSEL = 0x00; 
	GPIOB->DEN |= 0x000000ff;              // detrmining the pins direction 
  GPIOB->DATA |= 0x00000000;
	GPIOB -> PDR = 0xff; 	
}
void PORTD_INIT(void){

	SYSCTL->RCGCGPIO |= 0x00000008; 	// initialize clock                           // looping until clock is initiallized
	GPIOD->LOCK = 0x4C4F434B;
	GPIOD->AMSEL= 0x00;       				 // 3) disable analog function
  GPIOD->PCTL = 0x00000000;  				 // 4) GPIO clear bit PCTL 
	GPIOD->CR |= 0x000000ff;                                    // unlocking commit register for switch 1 & switch 2 
  GPIOD->DIR |= 0x00000000;                            // detrmining the output pins                                   
  GPIOD->AFSEL = 0x00; 
	GPIOD->DEN |= 0x000000ff;              // detrmining the pins direction 
  GPIOD->DATA |= 0x00000000;
	GPIOD -> PDR = 0xff; 	
	GPIOD->ICR = 0xff;     // Clear any Previous Interrupt
  GPIOD->IM |=0xff;      // Unmask the interrupts for PF0 and PF4	
	NVIC_EN0_R |= 0x08; 
	NVIC_PRI0_R |= 0xe000; 
}

void PORTA_INIT(void){
	SYSCTL_RCGCGPIO_R |= 0x00000001;                                     // initialize clock
	while((SYSCTL_PRGPIO_R&0x00000001) == 0){}                           // looping until clock is initiallized
	GPIO_PORTA_CR_R |= 0x000000ff;                                    // unlocking commit register for switch 1 & switch 2 
  GPIO_PORTA_DIR_R |= 0x0000000c;                            // detrmining the output pins                                   
  GPIO_PORTA_DEN_R |= 0x000000ff;              // detrmining the pins direction 
  GPIO_PORTA_DATA_R |= 0x00000000;
	GPIOA -> PUR = 0x80; 
}


/*----------------------------Port-F handler--------------------------------------------*/
void GPIOF_Handler(void){
 uint32_t pinFlags;
 pinFlags = GPIO_PORTF_MIS_R;
//--------------------------Jammer Button--------------------
	 if(pinFlags & (1 << 0)){
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR(xJamPressedSemaphore,&xHigherPriorityTaskWoken);
	GPIOF->ICR = 0xff;        // clear the interrupt flag of PORTF 
	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
  }
}

/*----------------------------Port-D handler--------------------------------------------*/
void GPIOD_isrHandler(void){
 uint32_t pinFlags;
 pinFlags = GPIO_PORTD_MIS_R;
/*--------------------------Limit Switch up Button--------------------*/
	 if(pinFlags & (1 << 0)){
		 GPIOD->ICR = 0xff;  
      xQueueSendFromISR(xMotorQueue,&(uint32_t){OFF},0);	// 
  }
/*----------------------------Limit Switch down Button-------------------*/
	 if (pinFlags & (1 << 1)){
		 GPIOD->ICR = 0xff;  
			xQueueSendFromISR(xMotorQueue,&(uint32_t){OFF},0);	
 }
}
