
#include "sysinc.h"
#include "AS5048A.h"

//主函数
int main(void){

	NVIC_init();
	SYSTICK_DelayInit();	    	 //延时函数初始化	  	
	AS5048A_QH_Init();
	
	UART_QuickInit(HW_UART1, 115200, 2, 2, ENABLE);  //初始化串口1，波特率115200
	printf("init over\n");
	
	while(1){
		printf("val = %d \n",Read_As5048A_QH_Value(CMD_ANGLE));
		SYSTICK_DelayMs(1000);
	}
	
}
