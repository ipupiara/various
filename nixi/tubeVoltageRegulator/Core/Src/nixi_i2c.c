/*
 * nixi_i2c.c
 *
 *  Created on: Dec 30, 2021
 *      Author: Brigitte
 */

#include <main.h>

#define i2cUseDma

I2C_HandleTypeDef hi2c1;

#ifdef i2cUseDma
DMA_HandleTypeDef hdma_i2c1_rx;
DMA_HandleTypeDef hdma_i2c1_tx;
#endif



typedef enum  {
	sendI2c = 0,
	receiveI2c,
} jobTypes;


typedef struct
{
	jobTypes  jobType;
	uint8_t*  buffer;
	uint8_t	amtChars;
	uint8_t   bufferCnt;
	uint8_t   address;
} i2cJobDataType;

i2cJobDataType i2cJobData;


void i2cFinishedOk()
{
//	setI2cJobSema();
}

void incDMAErrorCounter(DMA_HandleTypeDef *hdma)
{
//	if (__HAL_DMA_GET_FLAG(hdma,__HAL_DMA_GET_TE_FLAG_INDEX(hdma))) {
//		++ teCounter;
//	}
//	if (__HAL_DMA_GET_FLAG(hdma,__HAL_DMA_GET_FE_FLAG_INDEX(hdma))) {
//		++ feCounter;
//	}
//	if (__HAL_DMA_GET_FLAG(hdma,__HAL_DMA_GET_DME_FLAG_INDEX(hdma))) {
//		++ dmeCounter;
//	}
}


void i2cError(uint8_t err)
{
	 //log error
//	 transmitErrorCollectorInt8u = err;
//	 setI2cJobSema();

}


//INT8U  dmaIsr(DMA_HandleTypeDef *hdma)
//{
//	INT8U res = 0x00;
//	DMA_Base_Registers *regs = (DMA_Base_Registers *)hdma->StreamBaseAddress;
//	res = (INT8U)((regs->ISR >> ((DMA_HandleTypeDef *)hdma)->StreamIndex) & 0x3FU);
//	return res;
//  todo find a solution for this for stm32F103
//}

#ifdef i2cUseDma

void DMA1_Stream7_IRQHandler(void)
{


	if (__HAL_DMA_GET_FLAG(&hdma_i2c1_rx,DMA_FLAG_TC7) != 0)  {
//		transferBuffer();
//		i2cFinishedOk();
		__HAL_DMA_CLEAR_FLAG(&hdma_i2c1_rx,DMA_FLAG_TC7);
	}


    if (__HAL_DMA_GET_FLAG(&hdma_i2c1_rx,DMA_FLAG_HT7 ) != 0)  {
//    	transferBuffer();
    	__HAL_DMA_CLEAR_FLAG(&hdma_i2c1_rx,DMA_FLAG_HT7 );
    }

	if ((__HAL_DMA_GET_FLAG(&hdma_i2c1_rx,DMA_FLAG_TE7)))
//								| (__HAL_DMA_GET_FLAG(&hdma_i2c1_rx,DMA_FLAG_FEIF0_4))
	//							| (__HAL_DMA_GET_FLAG(&hdma_i2c1_rx,DMA_FLAG_DMEIF0_4)))
			{
		incDMAErrorCounter(&hdma_i2c1_rx);
//		i2cError(dmaIsr(&hdma_i2c1_rx ));
		__HAL_DMA_CLEAR_FLAG(&hdma_i2c1_rx,DMA_FLAG_TE7);
//		__HAL_DMA_CLEAR_FLAG(&hdma_i2c1_rx,DMA_FLAG_FEIF0_4);
//		__HAL_DMA_CLEAR_FLAG(&hdma_i2c1_rx,DMA_FLAG_DMEIF0_4);
	}

}

void DMA1_Stream6_IRQHandler(void)
{
//	CPU_SR_ALLOC();
////	INT8U err = OS_ERR_NONE;
//
//	CPU_CRITICAL_ENTER();
//	OSIntEnter();           /* Tell OS that we are starting an ISR           */
	//  todo find a way to handle timeout without rtos
//	CPU_CRITICAL_EXIT();

	if (__HAL_DMA_GET_FLAG(&hdma_i2c1_tx,DMA_FLAG_TC6) != 0)  {
		i2cFinishedOk();
		__HAL_DMA_CLEAR_FLAG(&hdma_i2c1_tx,DMA_FLAG_TC6);
	}


    if (__HAL_DMA_GET_FLAG(&hdma_i2c1_tx,DMA_FLAG_HT6) != 0)  {
//    	transferBuffer();
    	__HAL_DMA_CLEAR_FLAG(&hdma_i2c1_tx,DMA_FLAG_HT6);
    }

	if ((__HAL_DMA_GET_FLAG(&hdma_i2c1_tx,DMA_FLAG_TE6)))
//								| (__HAL_DMA_GET_FLAG(&hdma_i2c1_tx,DMA_FLAG_FEIF2_6))
//								| (__HAL_DMA_GET_FLAG(&hdma_i2c1_tx,DMA_FLAG_DMEIF2_6)))
		{
		incDMAErrorCounter(&hdma_i2c1_tx);
//		i2cError(dmaIsr(&hdma_i2c1_tx));
		__HAL_DMA_CLEAR_FLAG(&hdma_i2c1_tx,DMA_FLAG_TE6);
//		__HAL_DMA_CLEAR_FLAG(&hdma_i2c1_tx,DMA_FLAG_FEIF2_6);
//		__HAL_DMA_CLEAR_FLAG(&hdma_i2c1_tx,DMA_FLAG_DMEIF2_6);
	}
//	 OSIntExit();
}

void HAL_I2C_DmaInit(I2C_HandleTypeDef* hi2c)
{
	/**
	* Enable DMA controller clock
	*/
	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();


    hdma_i2c1_rx.Instance = DMA1_Channel7;
    hdma_i2c1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_i2c1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_i2c1_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_i2c1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_i2c1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_i2c1_rx.Init.Mode = DMA_NORMAL;
    hdma_i2c1_rx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_i2c1_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(hi2c,hdmarx,hdma_i2c1_rx);

    /* I2C1_TX Init */
    hdma_i2c1_tx.Instance = DMA1_Channel6;
    hdma_i2c1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_i2c1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_i2c1_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_i2c1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_i2c1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_i2c1_tx.Init.Mode = DMA_NORMAL;
    hdma_i2c1_tx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_i2c1_tx) != HAL_OK)
    {
      Error_Handler();
    }


    __HAL_LINKDMA(hi2c,hdmatx,hdma_i2c1_tx);

    /* I2C1 interrupt Init */
	HAL_NVIC_SetPriority(I2C1_EV_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
	HAL_NVIC_SetPriority(I2C1_ER_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);


	/* DMA interrupt init */
	/* DMA1_Channel6_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
	/* DMA1_Channel7_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);


}


#else

void sendNextI2CByte()
{
	if (i2cJobData.bufferCnt < i2cJobData.amtChars) {
		hi2c1.Instance->DR = i2cJobData.buffer[i2cJobData.bufferCnt];
		++i2cJobData.bufferCnt;
	}
}

void receiveNextI2CByte()
{
	if (i2cJobData.bufferCnt < i2cJobData.amtChars) {
			i2cJobData.buffer[i2cJobData.bufferCnt] = hi2c1.Instance->DR ;
			++i2cJobData.bufferCnt;
	}
}

#endif


/**
  * @brief This function handles I2C1 event interrupt.
  */

#define I2C_FLAG_NACKF  I2C_FLAG_AF

void I2C1_EV_IRQHandler(void)
{
	uint32_t itflags   = READ_REG(hi2c1.Instance->SR1);
#ifndef i2cUseDma
	if ((itflags & I2C_FLAG_TXE) != 0)   {
		sendNextI2CByte();
	}
	if ((itflags & I2C_FLAG_RXNE) != 0)   {
		receiveNextI2CByte();
	}
#endif
	if ((itflags & I2C_FLAG_TRA) != 0)  {      //  todo check if  tra is the right flag....
		i2cFinishedOk();
	}
	if (((itflags & I2C_FLAG_STOPF) != 0)| ((itflags & I2C_FLAG_NACKF) != 0) )  {
		__HAL_I2C_CLEAR_FLAG(&hi2c1, I2C_FLAG_STOPF);
		__HAL_I2C_CLEAR_FLAG(&hi2c1, I2C_FLAG_NACKF);
		  i2cError(0x77);
	}
}

void I2C1_ER_IRQHandler(void)
{

	// copied from stm32f7xx_hal_i2d.c
	uint32_t itflags   = READ_REG(hi2c1.Instance->SR1);
//	uint32_t itsources = READ_REG(hi2c1.Instance->CR1);
	  /* I2C Bus error interrupt occurred ------------------------------------*/
	  if ((itflags & I2C_FLAG_BERR) != RESET)
	  {
	    __HAL_I2C_CLEAR_FLAG(&hi2c1, I2C_FLAG_BERR);
	  }

	  /* I2C Over-Run/Under-Run interrupt occurred ----------------------------------------*/
	  if ((itflags & I2C_FLAG_OVR) != RESET)
	  {
	    __HAL_I2C_CLEAR_FLAG(&hi2c1, I2C_FLAG_OVR);
	  }

	  /* I2C Arbitration Loss error interrupt occurred -------------------------------------*/
	  if ((itflags & I2C_FLAG_ARLO) != RESET)
	  {
	    __HAL_I2C_CLEAR_FLAG(&hi2c1, I2C_FLAG_ARLO);
	  }
	  i2cError(0x82);  //  todo implement refined error message with above details....

	  // next two ifs are just for debugging reasons
	  if ((itflags & I2C_FLAG_AF) != 0) {   //  should actually be named I2C_FLAG_NACKF. how this name ?
		  i2cError(0x69);
	  }
	  if ((itflags & I2C_FLAG_STOPF) != 0) {
		  i2cError(0x96);
	  }

}


void HAL_I2C_GpioInit(I2C_HandleTypeDef* hi2c)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(hi2c->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspInit 0 */

  /* USER CODE END I2C1_MspInit 0 */

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**I2C1 GPIO Configuration
    PB6     ------> I2C1_SCL
    PB7     ------> I2C1_SDA
    */

    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Peripheral clock enable */
    __HAL_RCC_I2C1_CLK_ENABLE();

  }

}



void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_I2C_GpioInit(&hi2c1);

  /* I2C1 interrupt Init */
	HAL_NVIC_SetPriority(I2C1_EV_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
	HAL_NVIC_SetPriority(I2C1_ER_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);

#ifdef i2cUseDma
	HAL_I2C_DmaInit(&hi2c1);
#endif

}



