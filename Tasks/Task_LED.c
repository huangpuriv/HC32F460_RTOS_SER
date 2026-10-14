#include "../Tasks/Task_LED.h"
#include "../PeripheralDriver/Hw_Uart4.h"
#include "System_PowerDown.h"
TaskHandle_t H_Task_LED;
#define STACKSIZE_LED	256   /* 1KB 栈: UART4_FrameProc() 里要做整帧回发 */
#define PRIORITY_TASKLED	(tskIDLE_PRIORITY+3)
static void Task_LED(void* param)
{
	User_Gpio_Init();	
	while(1)
		{
			LED0_Toggle();
		UART4_FrameProc();  /* poll data-port frame (DMA+RTO mode) */
		    vTaskDelay(10/portTICK_PERIOD_MS);
		}
}
uint8_t Task_LED_Start(void)
{
	xTaskCreate(Task_LED,(const char *)"LED Task",STACKSIZE_LED,NULL,PRIORITY_TASKLED,&H_Task_LED);
	return Ok;
}
