/*
 * nixi_i2c.c
 *
 *  Created on: Dec 30, 2021
 *      Author: Brigitte
 */


#include <main.h>
#include <string.h>
#include <nixi_i2c.h>
#include <screen.h>

//  todo receive handling of result string not yet implemented, just send for screen used so far

//#define i2cUseDma
//#define useErrorLine

#define I2C_FLAG_NACKF  I2C_FLAG_AF
#define isJobBusy  __HAL_I2C_GET_FLAG(&hi2c1,I2C_FLAG_BUSY)
#define isInReceiveJob(__HANDLE__) ( (__HAL_I2C_GET_FLAG((__HANDLE__),I2C_FLAG_TRA) == 0) && (isJobBusy))

I2C_HandleTypeDef hi2c1;


uint8_t i2cInitialized;
uint8_t i2cInitNeeded;
uint32_t i2cInitNeededCnt;

uint32_t errTotalCnt;
uint8_t busyCnt;

#ifdef debugging
	uint8_t i2cErrorString  [i2cErrorStringLength];
	void addToErrorString(char* stri )
	{
#ifdef useErrorLine
		uint8_t pos;
		uint8_t lenAdd = strlen((char*) stri);
		for (pos = 0; (pos < lenAdd) && (strlen((char*) i2cErrorString) < (i2cErrorStringLength -1)) ; ++ pos)  {
			i2cErrorString[strlen((char*) i2cErrorString)] = stri[pos];
		}
#endif
	}


#else
#define addToErrorString( stri ) UNUSED( stri)
#endif


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

void setCr1Bit(uint16_t bitMask)
{
	/*
	 *
	 * Note: When the STOP, START or PEC bit is set, the software must not perform any write access
to I2C_CR1 before this bit is cleared by hardware. Otherwise there is a risk of setting a
second STOP, START or PEC request.  (datsheet-> i2c -> registers -> cr1 at the end)
	 */

	if ((READ_BIT(hi2c1.Instance->CR1, I2C_CR1_STOP) == 0 )
			&& (READ_BIT(hi2c1.Instance->CR1, I2C_CR1_PEC ) == 0 )
			&& (READ_BIT(hi2c1.Instance->CR1, I2C_CR1_START ) == 0 ) ) {

			SET_BIT(hi2c1.Instance->CR1, bitMask);
	}
}

void enableI2c()
{
	 __HAL_I2C_ENABLE(&hi2c1);
}

void disableI2c()
{
	__HAL_I2C_DISABLE(&hi2c1);
}


void i2cStopTransmission(I2C_HandleTypeDef *hi2c)
{
	setCr1Bit( I2C_CR1_STOP );
	__HAL_I2C_DISABLE_IT(&hi2c1, I2C_IT_EVT | I2C_IT_BUF | I2C_IT_ERR);

}

void i2cSetDataIdle()
{
	i2cJobData.jobType = idleI2c;
}


void i2cFinishedOk()
{
	i2cStopTransmission(&hi2c1);

	if (i2cJobData.jobType == sendI2c) {
		i2cMessageSent = 1;
	}
	if (i2cJobData.jobType == receiveI2c)  {
		i2cMessageReceived = 1;
	}
	i2cSetDataIdle();
}

void i2cError(uint8_t err)
{
	i2cStopTransmission(&hi2c1);
//	disableI2c();
//	++errTotalCnt;
//	i2cInitNeeded = 1;
//	i2cInitialized = 0;
	i2cSetDataIdle();
}

void initErrorHandling()
{
	errTotalCnt = 0;
	busyCnt = 0;
#ifdef debugging
	memset(i2cErrorString,0,i2cErrorStringLength);
#endif

}

uint8_t isI2cBusy()
{
	return (__HAL_I2C_GET_FLAG(&hi2c1,I2C_FLAG_BUSY));
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
	setCr1Bit( I2C_CR1_START);
}

void i2cSendStop(I2C_HandleTypeDef *hi2c)
{
	setCr1Bit( I2C_CR1_STOP);
}

void writeAddressToDR()
{
	hi2c1.Instance->DR = (i2cJobData.address << 1);
	if (i2cJobData.jobType == receiveI2c) {
		hi2c1.Instance->DR |= 0x01;
	}
}

uint8_t establishContactAndRun()
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

	uint8_t res = 0;


	    if ((__HAL_I2C_GET_FLAG(&hi2c1, I2C_FLAG_BUSY) == 0) ) {

	    if ((hi2c1.Instance->CR1 & I2C_CR1_PE) != I2C_CR1_PE)
	    {
	    	enableI2c();
	    }

	    CLEAR_BIT(hi2c1.Instance->CR1, I2C_CR1_POS);

	    __HAL_I2C_ENABLE_IT(&hi2c1, I2C_IT_EVT | I2C_IT_BUF | I2C_IT_ERR);

	    SET_BIT(hi2c1.Instance->CR1, I2C_CR1_START);

	    return 1;
	  }
	return res;
}


uint8_t transmitI2cByteArray(uint8_t adr,uint8_t* pResultString,uint8_t amtChars, uint8_t doSend)
{
	uint8_t res = 0x00;

	if ((i2cInitialized == 1) && (! isI2cBusy()) && (i2cJobData.jobType == idleI2c) ) {          //&& (OSIntNesting > 0u))
		busyCnt = 0;
		i2cJobData.buffer = pResultString;
		i2cJobData.amtChars = amtChars;
		i2cJobData.bufferCnt = 0;
		i2cJobData.address = adr;
		if (doSend == sendI2c) {
			i2cJobData.jobType = sendI2c;
		} else {
			i2cJobData.jobType = receiveI2c;
			if (pResultString != 0) {
			memset(pResultString,0,amtChars);  // todo check if this work correct (not content of pointer variable is changed)
			}
		}
		establishContactAndRun();
		res = 1;
	}  else {
		++ busyCnt;
		if (busyCnt > 30) {
			i2cInitNeeded = 1;
		}
	}

	return res;
}

uint8_t sendI2cByteArray(uint8_t adr,uint8_t* pString,uint8_t amtChars)
{
	uint8_t res = 0;
	res =  transmitI2cByteArray(adr, pString, amtChars,sendI2c);
	return res;
}

uint8_t receiveI2cByteArray(uint8_t adr,uint8_t* pString,uint8_t amtChars)
{
	uint8_t res = 0;
	res =  transmitI2cByteArray(adr, pString, amtChars,receiveI2c);
	return res;
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


uint8_t isCurrentByteSecondLastByte()
{
	return  (i2cJobData.bufferCnt == (i2cJobData.amtChars - 1));
}

uint8_t isCurrentByteLastByte()
{
	return  (i2cJobData.bufferCnt == (i2cJobData.amtChars));
}

#endif

void I2C1_EV_IRQHandler(void)
{
	uint8_t res; UNUSED(res);
	//  see datasheet  I2C -> functional description -> master receiver
	if (__HAL_I2C_GET_FLAG(&hi2c1,I2C_FLAG_SB) != 0){
		writeAddressToDR();
	} else {
		if (__HAL_I2C_GET_FLAG(&hi2c1,I2C_FLAG_ADDR) != 0){
			__HAL_I2C_CLEAR_ADDRFLAG(&hi2c1);
			if ((isCurrentByteSecondLastByte()) && (isInReceiveJob(&hi2c1)) )   {
				CLEAR_BIT(hi2c1.Instance->CR1, I2C_CR1_ACK );
				setCr1Bit(I2C_CR1_STOP );
			}
		}
		if (__HAL_I2C_GET_FLAG(&hi2c1,I2C_FLAG_BTF)!= 0) {
			__HAL_I2C_CLEAR_FLAG(&hi2c1,I2C_FLAG_BTF);
//			if (isCurrentByteLastByte()) {
//				i2cFinishedOk();
//			}
		}
		if (__HAL_I2C_GET_FLAG(&hi2c1,I2C_FLAG_TXE) != 0)   {
			if (isCurrentByteLastByte()) {
				i2cFinishedOk();
			}  else {
				sendNextI2CByte();
			}
		} else
		if (__HAL_I2C_GET_FLAG(&hi2c1, I2C_FLAG_RXNE) != 0)   {
			if (isCurrentByteSecondLastByte() ){
				CLEAR_BIT(hi2c1.Instance->CR1, I2C_CR1_ACK );
				setCr1Bit( I2C_CR1_STOP ); //  send stop  ( or (re-)start)
			}
			receiveNextI2CByte();
		}
//#endif
		if (__HAL_I2C_GET_FLAG(&hi2c1, I2C_FLAG_STOPF) != 0)   {
			setCr1Bit(I2C_CR1_PE );  // just write something into cr1 for stop reset, strange reset anyhow...
		}
		if (__HAL_I2C_GET_FLAG(&hi2c1,I2C_FLAG_NACKF) != 0) {
		  __HAL_I2C_CLEAR_FLAG(&hi2c1, I2C_FLAG_NACKF);
		}

	}
}

void I2C1_ER_IRQHandler(void)
{
	  if (__HAL_I2C_GET_FLAG(&hi2c1,I2C_FLAG_BERR) != 0) 	  {
	    __HAL_I2C_CLEAR_FLAG(&hi2c1, I2C_FLAG_BERR);
	    addToErrorString("BR");
	    i2cError(0x51);
	  }  else
	  if (__HAL_I2C_GET_FLAG(&hi2c1,I2C_FLAG_OVR) != 0)  {
	    __HAL_I2C_CLEAR_FLAG(&hi2c1, I2C_FLAG_OVR);
	    addToErrorString("OV");
	    i2cError(0x52);
	  } else
	  if (__HAL_I2C_GET_FLAG(&hi2c1,I2C_FLAG_ARLO) != 0) {
	    __HAL_I2C_CLEAR_FLAG(&hi2c1, I2C_FLAG_ARLO);
	    addToErrorString("AL");
	    i2cError(0x53);
	  }  else
	  if (__HAL_I2C_GET_FLAG(&hi2c1,I2C_FLAG_STOPF) != 0) {
		  __HAL_I2C_CLEAR_FLAG(&hi2c1, I2C_FLAG_STOPF);
		  addToErrorString("SP");
		  i2cError(0x96);
	  }
	  if (__HAL_I2C_GET_FLAG(&hi2c1,I2C_FLAG_NACKF) != 0) {
		  __HAL_I2C_CLEAR_FLAG(&hi2c1, I2C_FLAG_NACKF);
		  addToErrorString("NK");
		  i2cError(0x69);
	  }  else {
		  addToErrorString("Eo");
		  i2cError(0x54);
	  }


}

void init_i2c1_hw(void)
{
	i2cSetDataIdle();
	GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**I2C1 GPIO Configuration
    PB6     ------> I2C1_SCL
    PB7     ------> I2C1_SDA
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    __HAL_RCC_I2C1_CLK_ENABLE();

	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 25000;
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

	__HAL_I2C_ENABLE_IT(&hi2c1,I2C_IT_BUF);
	__HAL_I2C_ENABLE_IT(&hi2c1,I2C_IT_EVT);
	__HAL_I2C_ENABLE_IT(&hi2c1,I2C_IT_ERR);

	HAL_NVIC_SetPriority(I2C1_EV_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
	HAL_NVIC_SetPriority(I2C1_ER_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);
	enableI2c();
}

void initI2c()
{
	i2cInitNeededCnt = 0;

	i2cInitNeeded = 0;
	i2cSetDataIdle();
	initErrorHandling();
	init_i2c1_hw();
	i2cInitialized = 1;
}

void i2cReInitAfterFailure()
{
	i2cInitialized = 0;
	++i2cInitNeededCnt;
	disableI2c();
//	char buff[10];
//	snprintf(buff,sizeof(buff),"%lu",errTotalCnt);
//	addToErrorString(buff);

	uint32_t cnt = uwTick;
	while (uwTick < cnt +5 ) {}  // just in emergency....
	hi2c1.Instance->CR1 |= I2C_CR1_SWRST;
	cnt = uwTick;
	while (uwTick < cnt +3 ) {}  // just in emergency....
	hi2c1.Instance->CR1 &= ~I2C_CR1_SWRST;
	enableI2c();

	i2cSetDataIdle();
	init_i2c1_hw();
	initScreen();
	i2cInitialized = 1;
}
