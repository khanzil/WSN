#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include "esp32-hal-cpu.h"
#include "esp_rom_sys.h"
// put function declarations here:

TaskHandle_t toNode;
SemaphoreHandle_t QueueSem;


#define nss 5          
#define rst 14       
#define dio0 4
#define LED 2      
 
// Các lệnh của gateway
#define LEDThresh 0x00
#define ACK 0x01
#define OTAA 0x02

#define Node1 0x01
#define Node2 0x02
#define Node3 0x03
#define Node4 0x04
#define Node5 0x05
#define Node6 0x06
#define Node7 0x07
#define Node8 0x08
#define Node9 0x09
#define Node10 0x0A
#define Node11 0x0B
#define GatewayAddress 0xFF
#define Broadcast 0x00

String DataOut = "";
String DataIn = "";

uint16_t Thresh1 = 30, Thresh2 = 80;
bool processdata = 0;
void LoRa_Init();
void Send_Data(String Data, uint8_t NodeAddressOut);
void LoRa_rxMode();
void Send_ACK(uint8_t address);
void Send_Thresh();
void Process_Data();
void onReceive(int packetSize){
  processdata = 1;
}
void shiftleft();


uint32_t AddQueue[22];
float TempQueue[11];
uint8_t idx = 0;

void NodeHandler(void *parameter){
  for(;;){
    if(processdata == 1){
      // Serial.print("Data Received at: ");
      // Serial.println(esp_timer_get_time());
      Process_Data();
      LoRa_rxMode();
      processdata = 0;
    }
    else delayMicroseconds(1);
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  while(!Serial);
  Serial.println("Serial Init succeed!");
  LoRa_Init();
  LoRa.onReceive(onReceive);
  LoRa_rxMode();
  pinMode(LED, OUTPUT);

  xTaskCreatePinnedToCore(NodeHandler,
       "toNode",10000,NULL,1,&toNode,1);
  delay(500);
  
  QueueSem = xSemaphoreCreateBinary();
  if (QueueSem == NULL){
    Serial.println("Semaphore Create Failed");
    while(1);
  }  
}

void loop() {
  // put your main code here, to run repeatedly:
    // if(processdata == 1){
    //   // Serial.print("Data Received at: ");
    //   // Serial.println(esp_timer_get_time());
    //   Process_Data();
    //   LoRa_rxMode();
    //   processdata = 0;
    // }
}   

// put function definitions here:

void LoRa_Init(){
  LoRa.setPins(nss, rst, dio0);
  LoRa.setFrequency(433E6);
  LoRa.setCodingRate4(5);
  LoRa.setTxPower(17);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setSpreadingFactor(7);
  LoRa.setPreambleLength(8);     
  LoRa.disableCrc();
  if(!LoRa.begin(433E6)){
    Serial.println("LoRa begin failed!");
    while(1);
  }
  else{
    Serial.println("LoRa begin succeed!");
  }
}

void Send_Data(String data, uint8_t NodeAddress){
  LoRa.beginPacket();
  LoRa.write(GatewayAddress);
  LoRa.write(NodeAddress);
  LoRa.print(data);
  LoRa.endPacket();
  Serial.print("Data Sent: ");
  Serial.println(data);
  // Serial.print("Data Sent at: ");
  // Serial.println(esp_timer_get_time());
}

void LoRa_rxMode(){
  DataIn = "";
  LoRa.receive();
}

void Send_ACK(uint8_t address){
  DataOut = "";
  DataOut += ACK;
  Send_Data(DataOut, address);
}

void Send_Thresh(){
  DataOut = "";
  DataOut += LEDThresh;
  DataOut += Thresh1;
  DataOut += Thresh2;
  Send_Data(DataOut, 0x00);
}

void Process_Data(){
  uint8_t Sender = LoRa.read();
  uint8_t Receiver = LoRa.read();
  uint32_t gettime = millis();

  if(Receiver != GatewayAddress){
    Serial.println("Wrong address");
    return;
  }

  while(LoRa.available())
    DataIn += (char)LoRa.read();

  Send_ACK(Sender);

  if(DataIn[2] == OTAA)
    Send_ACK(Sender);
  else
    Send_Thresh();

  Serial.println(DataIn);
  if(xSemaphoreTake( QueueSem, portMAX_DELAY )){
    AddQueue[2*idx] = Sender;
    AddQueue[2*idx+1] = gettime;
    TempQueue[idx] = DataIn.toFloat();
    idx++;
    xSemaphoreGive(QueueSem);
  };


  return ;
}

void shiftleft(){
  if(xSemaphoreTake(QueueSem, portMAX_DELAY)){
    for(uint8_t i = 0; i < idx-1; i++){
        AddQueue[2*i] = AddQueue[2*(i+1)];
        AddQueue[2*i + 1] = AddQueue[2*(i+1) + 1];
        TempQueue[i] = TempQueue[i+1];
    }
    idx--;
    xSemaphoreGive(QueueSem);
  };
}


