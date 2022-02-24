/*
 * nixi_i2c.h
 *
 *  Created on: Dec 30, 2021
 *      Author: Brigitte
 */

#ifndef INC_NIXI_I2C_H_
#define INC_NIXI_I2C_H


uint8_t resetOnError;
uint8_t  i2cTransmitErrorCollectorInt8u;
uint8_t i2cInitialized;
uint8_t i2cSec100MsgPending;

void MX_I2C1_Init(void);
uint8_t isI2cBusy();
uint8_t sendI2cByteArray(uint8_t adr,uint8_t* pString,uint8_t amtChars);
uint8_t receiveI2cByteArray(uint8_t adr,uint8_t* pResultString,uint8_t amtChars);

void initI2c();
void reInitI2cAfterError();
void enableI2c();

#endif /* INC_NIXI_I2C_H_ */
