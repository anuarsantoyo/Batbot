#ifndef __AS5048A_H
#define __AS5048A_H


#include "sysinc.h"


#define CMD_ANGLE          0x3FFF  
#define CMD_READ_MAG       0x3FFE 
#define CMD_READ_DIAG      0x3FFD  
#define CMD_NOP            0x0000  
#define CMD_CLEAR_ERROR    0x0001  
#define CMD_ProgramControl 0x0003
#define CMD_OTPHigh 0x0016
#define CMD_OTPLow 0x0017


void AS5048A_QH_Init(void);
uint16_t SPI_WriteByte(uint16_t TxData);
void Write_As5048A_Reg(uint16_t cmd,uint16_t value);
uint8_t parity_even(uint16_t v);
uint16_t Read_As5048A_Reg(uint16_t cmd);
uint16_t Read_As5048A_QH_Value(uint16_t cmd);
uint8_t Write_As5048A_ZeroPosition(void);//OTP Abandoned

#endif

