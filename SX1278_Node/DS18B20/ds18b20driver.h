#ifndef DS18B20DRIVER_H
#define DS18B20DRIVER_H

#include "main.h"

typedef struct 
{
  GPIO_TypeDef* GPIOx;
  uint16_t GPIO_Pin;
	uint8_t ROM[8];
	float temperature;
} DS18B20_Sensor;

#define ReadROM 0x33
#define MatchROM 0x55
#define SkipROM 0xCC
#define SearchROM 0xF0
#define AlarmSearch 0xEC
#define WriteScratchpad 0x4E
#define ReadScratchpad 0xBE
#define CopyScrathpad 0x48
#define ConvertT 0x44
#define RecallE2 0xB8
#define ReadPowerSupply 0xB4

#define Resolution12Bit 0x7F
#define Resolution11Bit 0x5F
#define Resolution10Bit 0x3F
#define Resolution9Bit  0x1F

#define Family_code 0x28

#define WriteOneDelay 16
#define WriteZeroDelay 60
#define ReadDelay 4



void Onewire_PinOutput(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void Onewire_PinInput(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);

void OneWire_WriteBit(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, uint8_t bit);
void OneWire_WriteByte(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, uint8_t byte);
uint8_t OneWire_ReadBit(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
uint8_t OneWire_ReadByte(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);

uint8_t DS18B20_Reset(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);

void DS18B20_ReadROM(DS18B20_Sensor *sensor);
void DS18B20_MatchROM(DS18B20_Sensor sensor);
void DS18B20_SkipROM(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void DS18B20_SearchROM(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void DS18B20_AlarmSearch(DS18B20_Sensor sensor);
void DS18B20_WriteScratchPad(DS18B20_Sensor sensor);
void DS18B20_ReadSratchPad(DS18B20_Sensor sensor);
void DS18B20_CopySratchPad(DS18B20_Sensor sensor);
void DS18B20_ConvertT(DS18B20_Sensor sensor);
void DS18B20_RecallE2(DS18B20_Sensor sensor);
uint8_t DS18B20_ReadPowerSupply(DS18B20_Sensor sensor);
uint8_t DS18B20_CRCCheck();

void DS18B20_SetResolution(DS18B20_Sensor sensor, uint8_t resolution);
void DS18B20_GetResolution(DS18B20_Sensor sensor, uint8_t *resolution);
void DS18B20_GetTemp(DS18B20_Sensor sensor, float *temperature);

#endif