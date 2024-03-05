#include "ds18b20driver.h"

uint8_t scratchpad[9];

void Onewire_PinOutput(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin){
	GPIO_InitTypeDef	Gpio_Init_Struct = {0};
	Gpio_Init_Struct.Mode = GPIO_MODE_OUTPUT_PP;
	Gpio_Init_Struct.Pull = GPIO_NOPULL;
	Gpio_Init_Struct.Speed = GPIO_SPEED_FREQ_LOW;
	Gpio_Init_Struct.Pin = GPIO_Pin;	
	HAL_GPIO_Init(GPIOx,&Gpio_Init_Struct);
}	

void Onewire_PinInput(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin){
	GPIO_InitTypeDef	Gpio_Init_Struct = {0};
	Gpio_Init_Struct.Mode = GPIO_MODE_INPUT;
	Gpio_Init_Struct.Pull = GPIO_NOPULL;
	Gpio_Init_Struct.Pin = GPIO_Pin;	
	HAL_GPIO_Init(GPIOx,&Gpio_Init_Struct);
}

void OneWire_WriteBit(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, uint8_t bit){
	Onewire_PinOutput(GPIOx, GPIO_Pin);
	if (bit){
		HAL_GPIO_WritePin(GPIOx, GPIO_Pin, 0);
		delay_us(WriteOneDelay);
		HAL_GPIO_WritePin(GPIOx, GPIO_Pin, 1);
		delay_us(WriteZeroDelay-WriteOneDelay+5);
	}
	else{
		HAL_GPIO_WritePin(GPIOx, GPIO_Pin, 0);
		delay_us(WriteZeroDelay);
		HAL_GPIO_WritePin(GPIOx, GPIO_Pin, 1);
		delay_us(5);
	}
}
void OneWire_WriteByte(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, uint8_t byte){
	for(uint8_t i = 0; i < 8; i++){
		OneWire_WriteBit(GPIOx, GPIO_Pin, (byte >> i)%2);	
	}
}

uint8_t OneWire_ReadBit(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin){
	Onewire_PinOutput(GPIOx, GPIO_Pin);
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, 0);
	delay_us(5);
	
	Onewire_PinInput(GPIOx, GPIO_Pin);
	delay_us(15);
	uint8_t result = HAL_GPIO_ReadPin(GPIOx, GPIO_Pin);
	delay_us(50);
	
	return result;
}
uint8_t OneWire_ReadByte(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin){
	uint8_t byte = 0;
	for (uint8_t i = 0; i < 8; i++){		
		byte |= (OneWire_ReadBit(GPIOx, GPIO_Pin) << i);
	}
	return byte;
}

uint8_t DS18B20_Reset(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin){
	Onewire_PinOutput(GPIOx, GPIO_Pin);
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, 0);
	delay_us(500);
	Onewire_PinInput(GPIOx, GPIO_Pin);
	delay_us(70);
	uint8_t i = HAL_GPIO_ReadPin(GPIOx, GPIO_Pin);
	delay_us(410);
	return i;
}

void DS18B20_ReadROM(DS18B20_Sensor *sensor){
	DS18B20_Reset((*sensor).GPIOx, (*sensor).GPIO_Pin);
	OneWire_WriteByte((*sensor).GPIOx, (*sensor).GPIO_Pin, ReadROM);
	for(uint8_t i = 0; i<8; i++)
		(*sensor).ROM[i] = OneWire_ReadByte((*sensor).GPIOx, (*sensor).GPIO_Pin);
}

void DS18B20_MatchROM(DS18B20_Sensor sensor){
	DS18B20_Reset(sensor.GPIOx, sensor.GPIO_Pin);
	OneWire_WriteByte(sensor.GPIOx, sensor.GPIO_Pin, MatchROM);
	for(uint8_t i=0; i<8; i++)
		OneWire_WriteByte(sensor.GPIOx, sensor.GPIO_Pin, sensor.ROM[i]);
}

void DS18B20_SkipROM(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin){
	DS18B20_Reset(GPIOx, GPIO_Pin);
	OneWire_WriteByte(GPIOx, GPIO_Pin, SkipROM);
}

void DS18B20_WriteScratchPad(DS18B20_Sensor sensor){
	OneWire_WriteByte(sensor.GPIOx, sensor.GPIO_Pin, WriteScratchpad);
	for(uint8_t i = 2; i <= 4; i++)
		OneWire_WriteByte(sensor.GPIOx, sensor.GPIO_Pin, scratchpad[i]);
	DS18B20_Reset(sensor.GPIOx, sensor.GPIO_Pin);
}

void DS18B20_ReadSratchPad(DS18B20_Sensor sensor){
	OneWire_WriteByte(sensor.GPIOx, sensor.GPIO_Pin, ReadScratchpad);
	for(uint8_t i = 0; i <= 8; i++)
		scratchpad[i] = OneWire_ReadByte(sensor.GPIOx, sensor.GPIO_Pin);
	DS18B20_Reset(sensor.GPIOx, sensor.GPIO_Pin);
}


void DS18B20_CopySratchPad(DS18B20_Sensor sensor){
	OneWire_WriteByte(sensor.GPIOx, sensor.GPIO_Pin, CopyScrathpad);
	while(OneWire_ReadBit(sensor.GPIOx, sensor.GPIO_Pin) == 0);
	DS18B20_Reset(sensor.GPIOx, sensor.GPIO_Pin);
}

void DS18B20_ConvertT(DS18B20_Sensor sensor){
	OneWire_WriteByte(sensor.GPIOx, sensor.GPIO_Pin, ConvertT);
	while(OneWire_ReadBit(sensor.GPIOx, sensor.GPIO_Pin) == 0);
	DS18B20_Reset(sensor.GPIOx, sensor.GPIO_Pin);
}

void DS18B20_RecallE2(DS18B20_Sensor sensor){
	OneWire_WriteByte(sensor.GPIOx, sensor.GPIO_Pin, RecallE2);
	while(OneWire_ReadBit(sensor.GPIOx, sensor.GPIO_Pin) == 0);
	DS18B20_Reset(sensor.GPIOx, sensor.GPIO_Pin);
}

uint8_t DS18B20_ReadPowerSupply(DS18B20_Sensor sensor){
	OneWire_WriteByte(sensor.GPIOx, sensor.GPIO_Pin, ReadPowerSupply);
	uint8_t result = OneWire_ReadBit(sensor.GPIOx, sensor.GPIO_Pin);
	DS18B20_Reset(sensor.GPIOx, sensor.GPIO_Pin);
	return result;
}

uint8_t DS18B20_CRCCheck(){
	uint8_t crc = 0;
	return crc;
}

void DS18B20_SetResolution(DS18B20_Sensor sensor, uint8_t resolution){
	DS18B20_SkipROM(sensor.GPIOx, sensor.GPIO_Pin);
	DS18B20_ReadSratchPad(sensor);
	do{
		scratchpad[4] = resolution;
		DS18B20_SkipROM(sensor.GPIOx, sensor.GPIO_Pin);
		DS18B20_WriteScratchPad(sensor);
		DS18B20_SkipROM(sensor.GPIOx, sensor.GPIO_Pin);
		DS18B20_ReadSratchPad(sensor);
	}while(scratchpad[4] != resolution);
	DS18B20_SkipROM(sensor.GPIOx, sensor.GPIO_Pin);
	DS18B20_CopySratchPad(sensor);
}
void DS18B20_GetTemp(DS18B20_Sensor sensor, float *temperature){
	DS18B20_MatchROM(sensor);	
	DS18B20_ConvertT(sensor);
	DS18B20_MatchROM(sensor);
	DS18B20_ReadSratchPad(sensor);
	
	(*temperature) = ((scratchpad[1] << 8) | scratchpad[0]) * 0.0625;

	return;
}





