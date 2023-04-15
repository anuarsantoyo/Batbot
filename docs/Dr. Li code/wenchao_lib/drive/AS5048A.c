/**
   ******************************************************************************
   * @file      AS5048A.c
   * @author    深圳市沁和智能科技有限公司
   * @version   V1.0
   * @date      2020.03.09
   * @brief     
   ******************************************************************************
   * @attention--SPI2
   * SCK  -- PB13
   * MISO -- PB14
   * MOSI -- PB15
   * CS   -- PB12
   ******************************************************************************
   */
	 
#include "AS5048A.h"
#include "stm32f10x.h"

uint8_t error_flag;


/**
 * @BRIF  AS5048a初始化 
*/
  
void AS5048A_QH_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef SPI_InitStructure;

	RCC_APB1PeriphClockCmd( RCC_APB1Periph_SPI2, ENABLE );
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA| RCC_APB2Periph_GPIOB, ENABLE);


	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12; //Set CS pin high to disable device
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_SetBits(GPIOB, GPIO_Pin_12);


	SPI_Cmd(SPI2, DISABLE);
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;;
	SPI_InitStructure.SPI_Mode                     = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPI2, &SPI_InitStructure);

	SPI_Cmd(SPI2, ENABLE);

	SYSTICK_DelayUs(500);
}


/**
 * @param[in]   TxData    收发的数据
 * @retval      SPI接收的数据
 */
uint16_t SPI_WriteByte(uint16_t TxData)
{
	GPIO_ResetBits(GPIOB, GPIO_Pin_12);

	SYSTICK_DelayUs(1);

	SPI_I2S_SendData(SPI2, TxData); 
		
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET);

	SYSTICK_DelayUs(1);

	GPIO_SetBits(GPIOB, GPIO_Pin_12);

	SYSTICK_DelayUs(1);
	return SPI_I2S_ReceiveData(SPI2); 
}


// Calculates even parity of 16it value, returns 1 (odd) or 0 (even)
uint8_t parity_even(uint16_t v)
{
      if(v == 0) return 0;


      v ^= v >> 8;
      v ^= v >> 4;
      v ^= v >> 2;
      v ^= v >> 1;
      return v & 1;
}

/**
 * @function    读取寄存器值
 * @param[in]   cmd    寄存器地址
 * @retval      该地址的值
 */
uint16_t Read_As5048A_Reg(uint16_t cmd)
{
	uint16_t data = 0;
	uint16_t res;
	uint16_t command = 0x4000;// PAR=0 R/W=R
	command = command | cmd;
	command |= ((uint16_t)parity_even(command)<<15);//Add a parity bit on the the MSB
				SPI_WriteByte(command);

	command = 0x4000;// PAR=0 R/W=R
	command = command | CMD_NOP;
	command |= ((uint16_t)parity_even(command)<<15);//Add a parity bit on the the MSB
				res = SPI_WriteByte(command);
	error_flag = 1;

	if ((res & (1 << 14)) == 0){
		data = (res & 0x3FFF);
		error_flag = (parity_even(data) ^ (res >> 15));
	}
	else{
		command = 0x4000;
		command = command | CMD_CLEAR_ERROR;
		command |= ((uint16_t)parity_even(command)<<15);//Add a parity bit on the the MSB
		SPI_WriteByte(command);
	}
  return data;
}

/**
 * @function    给寄存器写数据
 * @param[in]   cmd    寄存器地址
 * @param[in]   value  写入的数据
 * @retval      void
 */
void Write_As5048A_Reg(uint16_t cmd,uint16_t value)
{
	uint16_t data = 0;
	uint16_t res;
	uint16_t command = 0x0000; // PAR=0 R/W=W

	command = command | cmd;
	command |= ((uint16_t)parity_even(command)<<15);//Add a parity bit on the the MSB
	SPI_WriteByte(command);


	command = 0x0000; // PAR=0 R/W=W
	command = command | value;
	command |= ((uint16_t)parity_even(command)<<15);//Add a parity bit on the the MSB
	SPI_WriteByte(command);

	command = 0x4000;// PAR=0 R/W=R
	command = command | CMD_NOP;
	command |= ((uint16_t)parity_even(command)<<15);//Add a parity bit on the the MSB
	res = SPI_WriteByte(command);
	error_flag = 1;

	if ((res & (1 << 14)) == 0){
			data = (res & 0x3FFF);
			error_flag = (parity_even(data) ^ (res >> 15));
	}
	else{
			command = 0x4000;
			command = command | CMD_CLEAR_ERROR;
			command |= ((uint16_t)parity_even(command)<<15);//Add a parity bit on the the MSB
			SPI_WriteByte(command);
  }
}


/**
 * @function    读取AS5048A数据
 * @param[in]   cmd    寄存器地址
 * @retval      角度信息
 */
uint16_t Read_As5048A_QH_Value(uint16_t cmd)
{
	uint16_t val;
	val = Read_As5048A_Reg(cmd);
	if(error_flag)
	{
	val = 0;
	}
	return val;
}


/************************************
OTP Write Zero Position: 0 for No error
1. Read angle information
2. Set the Programming Enable bit in the OTP control register
3. Write previous read angle position into OTP zero position register
4. Read back for verification the zero position register data
5. Set the Burn bit to start the automatic programming procedure
6. Read angle information (equals to 0)
7. Set the Verify bit to load the OTP data again into the internal registers with modified threshold comparator levels
8. Read angle information (equals to 0)
******************************************/
uint8_t Write_As5048A_ZeroPosition()
{
	uint16_t Angle_val;
	uint8_t Angle_High,Angle_Low;
	uint16_t cmd;
	Angle_val = Read_As5048A_QH_Value(CMD_ANGLE);
	cmd=Read_As5048A_QH_Value(CMD_ProgramControl);

	Write_As5048A_Reg(CMD_ProgramControl,0x01);
	SYSTICK_DelayUs(10);
	cmd=Read_As5048A_QH_Value(CMD_ProgramControl);
	SYSTICK_DelayUs(10);
	Write_As5048A_Reg(CMD_OTPHigh,Angle_val>>8);
	SYSTICK_DelayUs(10);
	Write_As5048A_Reg(CMD_OTPLow,Angle_val&0xFF);
	SYSTICK_DelayUs(10);
	Angle_High=Read_As5048A_QH_Value(CMD_OTPHigh);
	SYSTICK_DelayUs(10);
	Angle_Low=Read_As5048A_QH_Value(CMD_OTPLow);
	SYSTICK_DelayUs(10);
	if(Angle_High != (uint8_t)(Angle_val>>8))
	return 1;
	if(Angle_Low != (uint8_t)(Angle_val&0xFF))
	return 2;

	Write_As5048A_Reg(CMD_ProgramControl,0x09);
	SYSTICK_DelayUs(10);
	cmd=Read_As5048A_QH_Value(CMD_ProgramControl);
	SYSTICK_DelayUs(10);
	Angle_val = Read_As5048A_QH_Value(CMD_ANGLE);
	SYSTICK_DelayUs(10);
	if(Angle_val != 0 )
	return 3;

	Write_As5048A_Reg(CMD_ProgramControl,0x49);
	SYSTICK_DelayUs(10);
	cmd=Read_As5048A_QH_Value(CMD_ProgramControl);
	SYSTICK_DelayUs(10);
	Angle_val = Read_As5048A_QH_Value(CMD_ANGLE);
	SYSTICK_DelayUs(10);
	if(Angle_val != 0 )
	return 4;

	return 0;
}
