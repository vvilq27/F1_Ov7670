/*
 * sccb.c
 *
 *  Created on: Jan 7, 2021
 *      Author: vvilq
 */

#include "main.h"

#define SIO_CLOCK_DELAY 2

// B7 data ; B6 clk


__STATIC_INLINE void delayUs(volatile uint32_t microseconds)
{
 uint32_t clk_cycle_start = DWT->CYCCNT;
 /* Go to number of cycles for system */
 microseconds *= (HAL_RCC_GetHCLKFreq() / 1000000);
 /* Delay till end */
 while ((DWT->CYCCNT - clk_cycle_start) < microseconds);
}

void InitSCCB(void) //SCCB Initialization
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, SET);
}

void StartSCCB(void) //SCCB Start
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, SET);
//  delayUs(SIO_CLOCK_DELAY);

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, SET);
  delayUs(SIO_CLOCK_DELAY);

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, RESET);
//  delayUs(SIO_CLOCK_DELAY);

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, RESET);
  delayUs(SIO_CLOCK_DELAY);
}

void StopSCCB(void) //SCCB Stop
{
  //Serial.println("StopSCCB");

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, RESET);
//  delayUs(SIO_CLOCK_DELAY);

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, SET);
  delayUs(SIO_CLOCK_DELAY);

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, SET);
  delayUs(SIO_CLOCK_DELAY);
}

uint8_t SCCBWrite(uint8_t m_data)
{
  unsigned char j;
  uint8_t success;

  for ( j = 0; j < 8; j++ ) //Loop transmit data 8 times
  {

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, RESET);

	if( (m_data<<j) & 0x80 )
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, SET);
	else
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, RESET);

    delayUs(SIO_CLOCK_DELAY);

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, SET);
    delayUs(SIO_CLOCK_DELAY);
  }

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, RESET);

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  GPIO_InitStruct.Pin = GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  delayUs(SIO_CLOCK_DELAY);

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, SET);
  delayUs(SIO_CLOCK_DELAY);

  if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7))
    success= 1;
  else
    success= 0;

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, RESET);
//  delayUs(SIO_CLOCK_DELAY);

  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  return success;
}

uint8_t SCCBWriteNACK(uint8_t m_data)
{
  unsigned char j;
  uint8_t success;

  for ( j = 0; j < 8; j++ ) //Loop transmit data 8 times
  {

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, RESET);

	if( (m_data<<j) & 0x80 )
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, SET);
	else
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, RESET);

    delayUs(SIO_CLOCK_DELAY);

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, SET);
    delayUs(SIO_CLOCK_DELAY);
  }

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, RESET);

  delayUs(SIO_CLOCK_DELAY);

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, SET);
  delayUs(SIO_CLOCK_DELAY);

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, RESET);

//  delayUs(SIO_CLOCK_DELAY);

  return success;
}

uint8_t WriteOV7670(char regID, char regDat)
{
	StartSCCB();
	SCCBWrite(0x42);
//	if( ! SCCBWrite(0x42) )
//	{
//  		StopSCCB();
//		return 0;
//	}

	delayUs(SIO_CLOCK_DELAY);

	SCCBWrite(regID);
//  	if( ! SCCBWrite(regID) )
//	{
//		StopSCCB();
//		return 0;
//	}
	delayUs(SIO_CLOCK_DELAY);
	SCCBWrite(regDat);
//  	if( ! SCCBWrite(regDat) )
//	{
//		StopSCCB();
//		return 0;
//	}

  	StopSCCB();

  	return 1;
}

uint8_t ReadOV7670(char regID)
{
	StartSCCB();
	SCCBWrite(0x42);
//	if( ! SCCBWrite(0x43) )
//	{
//  		StopSCCB();
//		return 0;
//	}

	delayUs(SIO_CLOCK_DELAY);

	SCCBWrite(regID);
//  	if( ! SCCBWrite(regID) )
//	{
//		StopSCCB();
//		return 0;
//	}
	delayUs(SIO_CLOCK_DELAY);
//	StopSCCB();
	StartSCCB();
	delayUs(SIO_CLOCK_DELAY);

	SCCBWrite(0x43) ;

//	if( ! SCCBWrite(0x43) )
//	{
//		StopSCCB();
//		return 0;
//	}

	GPIO_InitTypeDef GPIO_InitStruct = {0};

	  GPIO_InitStruct.Pin = GPIO_PIN_7;
	    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	    GPIO_InitStruct.Pull = GPIO_NOPULL;
	    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	delayUs(SIO_CLOCK_DELAY);

	for(uint8_t k = 0; k < 10 ; k++){
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, RESET);
		delayUs(SIO_CLOCK_DELAY);

		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, SET);
		delayUs(SIO_CLOCK_DELAY);
	}

  	StopSCCB();


  	GPIO_InitStruct.Pin = GPIO_PIN_7;
  	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  	  GPIO_InitStruct.Pull = GPIO_NOPULL;
  	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  	  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  	return 1;
}


uint8_t InitOV7670()
{
  char temp = 0x80;

  InitSCCB();

  WriteOV7670(0x12, temp);

//  if( !WriteOV7670(0x12, temp) ) //Reset SCCB
//  {
//    return 0;
//  }

//
//  HAL_Delay(2);
//
//  ReadOV7670(0x13);
//  HAL_Delay(2);
//
//  ReadOV7670(0x11);
//  HAL_Delay(2);
//
//  ReadOV7670(0x16);

  return 1;
}
