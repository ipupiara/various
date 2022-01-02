/*
 * nixi_i2c.c
 *
 *  Created on: Dec 30, 2021
 *      Author: Brigitte
 */

#include <main.h>
#include <string.h>
#include <nixi_i2c.h>

//#define i2cUseDma
#define I2C_FLAG_NACKF  I2C_FLAG_AF

I2C_HandleTypeDef hi2c1;

uint8_t  transmitErrorCollectorInt8u;
uint8_t  jobSemSet;

#ifdef i2cUseDma
DMA_HandleTypeDef hdma_i2c1_rx;
DMA_HandleTypeDef hdma_i2c1_tx;
#endif



typedef enum  {
	sendI2c = 0,
	receiveI2c,
	idleI2c
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

void i2cSetDataIdle()
{
	i2cJobData.jobType = idleI2c;
	i2cFinished = 1;   // todo change to express the event character
	jobSemSet = 1;
}

void i2cFinishedOk()
{
	i2cSetDataIdle();
//	setI2cJobSema();
}

void i2cError(uint8_t err)
{
	i2cSetDataIdle();
	 //log error
//	 transmitErrorCollectoruint8_t = err;
//	 setI2cJobSema();

}

uint8_t isI2cIdle()
{
	return i2cJobData.jobType == idleI2c;
}

uint8_t isI2cReceiving()
{
	return i2cJobData.jobType == receiveI2c;
}

uint8_t isI2cSending()
{
	return i2cJobData.jobType == sendI2c;
}

void enableI2c()
{
	 __HAL_I2C_ENABLE(&hi2c1);
}

void disableI2c()
{
	__HAL_I2C_DISABLE(&hi2c1);
}

// structure copied from stm32f7xx_hal_dma.c
typedef struct
{
  __IO uint32_t ISR;   /*!< DMA interrupt status register */
  __IO uint32_t Reserved0;
  __IO uint32_t IFCR;  /*!< DMA interrupt flag clear register */
} DMA_Base_Registers;


void clearDmaInterruptFlags(DMA_HandleTypeDef *hdma)
{
//	DMA_Base_Registers *regs = (DMA_Base_Registers *)hdma->StreamBaseAddress;
//	regs->IFCR = 0x3FU << ((DMA_HandleTypeDef *)hdma)->StreamIndex;

}


// structure copied from stm32f7xx_hal_dma.c
void DMA_SetTransferConfig(DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength)
{
  /* Clear DBM bit */
//  hdma->Instance->CR &= (uint32_t)(~DMA_SxCR_DBM);
//
//  /* Configure DMA Stream data length */
//  hdma->Instance->NDTR = DataLength;
//
//  /* Memory to Peripheral */
//  if((hdma->Init.Direction) == DMA_MEMORY_TO_PERIPH)
//  {
//    /* Configure DMA Stream destination address */
//    hdma->Instance->PAR = DstAddress;
//
//    /* Configure DMA Stream source address */
//    hdma->Instance->M0AR = SrcAddress;
//  }
//  /* Peripheral to Memory */
//  else
//  {
//    /* Configure DMA Stream source address */
//    hdma->Instance->PAR = SrcAddress;
//
//    /* Configure DMA Stream destination address */
//    hdma->Instance->M0AR = DstAddress;
//  }
}

void i2cSendStart(I2C_HandleTypeDef *hi2c)
{
//	WRITE_REG(hi2c->Instance->CR2, ((uint32_t)1U << I2C_CR2_START_Pos));
}

void i2cSendStop(I2C_HandleTypeDef *hi2c)
{
//	WRITE_REG(hi2c->Instance->CR2, ((uint32_t)1U << I2C_CR2_STOP_Pos));
}

void i2cTransferConfig(I2C_HandleTypeDef *hi2c,  uint16_t DevAddress, uint8_t Size,  uint8_t Request)
{

//  todo write the address into the message , read status register and handle dr empty event
}

void writeAddressToDR()
{
	hi2c1.Instance->DR = (i2cJobData.address << 1);
	if (i2cJobData.jobType == receiveI2c) {
		hi2c1.Instance->DR |= 0x01;
	}
}

void establishContactAndRun()
{
#ifdef i2cUseDma
	if (i2cJobData.jobType == sendI2c) {
		DMA_SetTransferConfig(&hdma_i2c1_tx,(uint32_t)i2cJobData.buffer,(uint32_t)&hi2c1.Instance->DR,i2cJobData.amtChars);
		clearDmaInterruptFlags(&hdma_i2c1_tx);
		__HAL_DMA_ENABLE(&hdma_i2c1_tx);
	} else {
		DMA_SetTransferConfig(&hdma_i2c1_rx,(uint32_t)&hi2c1.Instance->DR,(uint32_t)i2cJobData.buffer,i2cJobData.amtChars);
		clearDmaInterruptFlags(&hdma_i2c1_rx);
		__HAL_DMA_ENABLE(&hdma_i2c1_rx);
	}
#endif

	i2cTransferConfig(&hi2c1,i2cJobData.address,i2cJobData.amtChars,(i2cJobData.jobType == receiveI2c ? 1:0));

	i2cSendStart(&hi2c1);
}

uint8_t pollForReady(uint8_t adr, uint8_t delay)
{
	int8_t res = 0xFF;
//	int8_t resOnErrStack = resetOnError;
//	resetOnError = 0;
	uint8_t dummyBuffer [1];

	while (res != 0) {
		sendI2cByteArray(adr,&dummyBuffer[0],0, delay);  // do just a very short delay if desired
//		resetOnError = resOnErrStack;
	}
    return res;
}
//  todo should eventually better be transferred into eeprom code


uint8_t transmitI2cByteArray(uint8_t adr,uint8_t* pResultString,uint8_t amtChars, uint8_t doSend, uint8_t delayMs)
{
	uint8_t res = 0xFF;

	if ((i2cInitialized == 1) ) {          //&& (OSIntNesting > 0u))
//		uint8_t semErr;
//		OSSemPend(i2cResourceSem, 2803, &semErr);
//		if (semErr == OS_ERR_NONE) {
		transmitErrorCollectorInt8u = 0;
//			OSSemSet(i2cJobSem,0,&semErr);  // debug: be sure it was not set multiple times at last end of transfer..
			jobSemSet = 0;
			i2cJobData.buffer = pResultString;
			i2cJobData.amtChars = amtChars;
			i2cJobData.bufferCnt = 0;
			i2cJobData.address = adr;
			if (doSend == 1) {
				i2cJobData.jobType = sendI2c;
			} else {
				i2cJobData.jobType = receiveI2c;
				if (pResultString != 0) {
				memset(pResultString,0,amtChars);  // todo check if this work correct (not content of pointer variable is changed)
				}
			}

			establishContactAndRun();

	//		OSSemPend(i2cJobSem, 0, &semErr);
//			if (semErr != OS_ERR_NONE) {
//				transmitErrorCollectoruint8_t = semErr;
//			}
			if (delayMs > 0) {
//				OSTimeDlyHMSM(0, 0, 0, delayMs);
			}
			//  todo wait until data written into eeprom memory
//			OSSemSet(i2cResourceSem, 1, &semErr);
			res = transmitErrorCollectorInt8u;
		//}  else {
		//	res = semErr;
	}
	return res;
}

//  todo delay should not be done on transmit level on i2c side,
//  but on client side that needs the delay
//  else traffic will be blocked for all
uint8_t sendI2cByteArray(uint8_t adr,uint8_t* pString,uint8_t amtChars, uint8_t delayMs)
{
	return transmitI2cByteArray(adr, pString, amtChars, 1, delayMs);
}

uint8_t receiveI2cByteArray(uint8_t adr,uint8_t* pString,uint8_t amtChars, uint8_t delayMs)
{
	return transmitI2cByteArray(adr, pString, amtChars, 0, delayMs);
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



//uint8_t  dmaIsr(DMA_HandleTypeDef *hdma)
//{
//	uint8_t res = 0x00;
//	DMA_Base_Registers *regs = (DMA_Base_Registers *)hdma->StreamBaseAddress;
//	res = (uint8_t)((regs->ISR >> ((DMA_HandleTypeDef *)hdma)->StreamIndex) & 0x3FU);
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
////	uint8_t err = OS_ERR_NONE;
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

uint8_t isMessageTransferred()
{
	return  (i2cJobData.bufferCnt >= i2cJobData.amtChars);
}

#endif


/**
  * @brief This function handles I2C1 event interrupt.
  */



void I2C1_EV_IRQHandler(void)
{

#ifndef i2cUseDma
	if (! isMessageTransferred())  {
		if (__HAL_I2C_GET_FLAG(&hi2c1,I2C_FLAG_BTF)) {
			__HAL_I2C_CLEAR_FLAG(&hi2c1,I2C_FLAG_BTF);
		}
		if (__HAL_I2C_GET_FLAG(&hi2c1,I2C_FLAG_TXE) != 0)   {
			sendNextI2CByte();
		} else
		if (__HAL_I2C_GET_FLAG(&hi2c1, I2C_FLAG_RXNE) != 0)   {
			receiveNextI2CByte();
		}
	}
#endif
	if (__HAL_I2C_GET_FLAG(&hi2c1,I2C_FLAG_SB) != 0){
		writeAddressToDR();
	}
	if (__HAL_I2C_GET_FLAG(&hi2c1,I2C_FLAG_ADDR) != 0){
		__HAL_I2C_CLEAR_ADDRFLAG(&hi2c1);
			// send receive bytes
	}
	if ((__HAL_I2C_GET_FLAG(&hi2c1,I2C_FLAG_TXE) != 0) &&( isMessageTransferred()))  {
		// send stop condition
		i2cFinishedOk();
	}

//	if ((itflags & I2C_FLAG_TRA) != 0)  {      //  todo check if  tra is the right flag....
//		i2cFinishedOk();
//	}
	if ((__HAL_I2C_GET_FLAG(&hi2c1, I2C_FLAG_STOPF) != 0)|| (__HAL_I2C_GET_FLAG(&hi2c1, I2C_FLAG_NACKF) != 0) )  {
		__HAL_I2C_CLEAR_FLAG(&hi2c1, I2C_FLAG_STOPF);
		__HAL_I2C_CLEAR_FLAG(&hi2c1, I2C_FLAG_NACKF);
		  i2cError(0x77);
	}
}

void I2C1_ER_IRQHandler(void)
{

	  if (__HAL_I2C_GET_FLAG(&hi2c1,I2C_FLAG_BERR) != 0)
	  {
	    __HAL_I2C_CLEAR_FLAG(&hi2c1, I2C_FLAG_BERR);
	  }
	  if (__HAL_I2C_GET_FLAG(&hi2c1,I2C_FLAG_OVR) != 0)
	  {
	    __HAL_I2C_CLEAR_FLAG(&hi2c1, I2C_FLAG_OVR);
	  }
	  if (__HAL_I2C_GET_FLAG(&hi2c1,I2C_FLAG_ARLO) != 0)
	  {
	    __HAL_I2C_CLEAR_FLAG(&hi2c1, I2C_FLAG_ARLO);
	  }
	  //  todo implement refined error message with above details....

	  if (__HAL_I2C_GET_FLAG(&hi2c1,I2C_FLAG_NACKF) != 0) {   //  should actually be named I2C_FLAG_NACKF. how this name ?
		  i2cError(0x69);
	  }
	  if (__HAL_I2C_GET_FLAG(&hi2c1,I2C_FLAG_STOPF) != 0) {
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

	i2cFinished = 0;
	i2cMsgPending = 0;
	i2cInitialized = 0;
	i2cSetDataIdle();

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
#else

#endif






	i2cInitialized = 1;
}



