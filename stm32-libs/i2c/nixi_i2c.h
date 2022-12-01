/*
 * nixi_i2c.h
 *
 *  Created on: Dec 30, 2021
 *      Author: Brigitte
 */

#ifndef INC_NIXI_I2C_H_
#define INC_NIXI_I2C_H

#define i2cErrorStringLength  80
#define debugging

extern uint8_t i2cErrorString  [i2cErrorStringLength];
extern uint8_t i2cInitialized;
extern uint8_t i2cInitNeeded;
extern uint32_t i2cInitNeededCnt;


uint8_t isI2cBusy();
uint8_t sendI2cByteArray(uint8_t adr,uint8_t* pString,uint8_t amtChars);
uint8_t receiveI2cByteArray(uint8_t adr,uint8_t* pResultString,uint8_t amtChars);

void initI2c();
void i2cReInitAfterFailure();

void I2C1_EV_IRQHandler(void);
void I2C1_ER_IRQHandler(void);

#endif /* INC_NIXI_I2C_H_ */
