
#include <cpu.h>
#include <main.h>
#include <string.h>
#include <nixi_i2c.h>
#include <humidTempSensor.h>

#define humidTempSensorI2cAddress 0x3c
#define byteAryMaxSz  12


typedef struct  {
	uint8_t len;
	uint8_t buffer [byteAryMaxSz];
} byteAryType;

byteAryType  btBuffer;
void humidTempTick()
{
	sendI2cByteArray(humidTempSensorI2cAddress,btBuffer.buffer ,btBuffer.len);
}



void initHumidTempSensor()
{
	memset(&btBuffer,0,sizeof(btBuffer));
}
