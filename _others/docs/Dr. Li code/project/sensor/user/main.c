
#include "sysinc.h"
#include "AS5048A.h"

//������
int main(void){

	NVIC_init();
	SYSTICK_DelayInit();	    	 //��ʱ������ʼ��	  	
	AS5048A_QH_Init();
	
	UART_QuickInit(HW_UART1, 115200, 2, 2, ENABLE);  //��ʼ������1��������115200
	printf("init over\n");
	
	while(1){
		printf("val = %d \n",Read_As5048A_QH_Value(CMD_ANGLE));
		SYSTICK_DelayMs(1000);
	}
	
}
