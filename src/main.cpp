#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <HTTPClient.h>
#include <AsyncJson.h>
#include <EnergyMeter.h>
// #include "driver/ledc.h"
#include "driver/pcnt.h"
#include "soc/pcnt_struct.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <HardwareSerial.h>
#include <SerialHandler.h>
#include <JsonHandler.h>
#include <DataDef.h>
#include <LedControl.h>
#include <RelayControl.h>
#include <WebServerHandler.h>
#include <ShiftRegister74HC595.h>
#include <ModbusClientRTU.h>
#include <ModbusBridgeWiFi.h>
#include <Vector.h>

#define READ_INTERVAL 20

JsonHandler jsonHandler(&Serial);
// SerialHandler serialHandler(&Serial);
SerialHandler serialHandler;
ModbusClientRTU MB;
ModbusBridgeWiFi MBbridge;
uint32_t request_time;
uint16_t port = 502;
bool data_ready = false;

const int numberOfShiftRegister = 8;
const int numberOfLed = 24;
const int din = 18; //data pin
const int stcp = 19; //latch pin
const int shcp = 21; //clock pin
ShiftRegister74HC595<numberOfShiftRegister> sr(din, shcp, stcp);

const int ledPin = 22; //led data pin
CRGB leds[numberOfLed];

WebServerHandler webServerHandler;
LineData lineData[64];
Command command[16];
Vector<Command> commandList(command);
ModbusRequest modbusRequestStorage[16];
Vector<ModbusRequest> modbusRequest(modbusRequestStorage);

RelayControl relayControl;
LedControl ledControl;

// Task handle and stack size
TaskHandle_t serialTaskHandle;
TaskHandle_t sendTaskHandle;
TaskHandle_t modbusTaskHandle;
const uint32_t stackSize = 4096;

QueueHandle_t serialQueueHandle;
const uint32_t queueSize = 10;

bool isReady;
unsigned int number = 0;
float coef_v = 0.125; // coefficient for voltage measurement, to get this value use the following formula coef u = voltage on voltmeter / reading frequency
float coef_i = 0.013;
float coef_p = 2.54;
int ledcPin[8] = {4, 13, 14, 15, 16, 17, 18, 19};
int pcntPin[8] = {21, 22, 23, 25, 26, 27, 32, 33};
int freqPin_1 = 21;
int freqPin_2 = 22;
int freqPin_3 = 23;
int cfPin = 19; // pin for active power value
int cf1Pin = 18; // pin for voltage or current RMS depending on selPin
int selPin = 23; // pin to switch measurement between voltage or current RMS (0 for current measurement, 1 for voltage measurement)
int16_t highLimit = 30000; // value for counter overflow

hw_timer_t *timer = NULL;

volatile bool isTriggered;
int mode;

portMUX_TYPE timerMux_1 = portMUX_INITIALIZER_UNLOCKED;

SemaphoreHandle_t myLock = xSemaphoreCreateMutex();

const int internalLed = 2; //internal led pin

// const char *ssid = "RnD_Sundaya";
// const char *password = "sundaya22";
const char *ssid = "POCO M4 Pro";
const char *password = "thomaspoco";
const String hostName = "RnD-BL-Board";

String dataString;

unsigned long lastReconnectMillis = 0;
int reconnectInterval = 5000;

AsyncWebServer server(80);

EnergyMeter energyMeter[8] = {
  EnergyMeter(pcntPin[2], -1, 0, 0),
  EnergyMeter(pcntPin[2], -1, 1, 0),
  EnergyMeter(pcntPin[2], -1, 2, 0),
  EnergyMeter(pcntPin[3], -1, 3, 0),
  EnergyMeter(pcntPin[4], -1, 4, 0),
  EnergyMeter(pcntPin[5], -1, 5, 0),
  EnergyMeter(pcntPin[6], -1, 6, 0),
  EnergyMeter(pcntPin[7], -1, 7, 0)
};

JsonData jsonData[8];
JsonHeader jsonHeader;
DataPack dataPack;
RelayStatus relayStatus[8];
String writeCommandStorage[12];
Vector <String> writeCommand(writeCommandStorage);

EnergyMeterData energyData[64]; //divided into a group for every 8
DTSU666_Data dtsu666Data[8];


/**
 * @brief ISR handler when PCNT event trigger
 *        @note This handler will decode which interrupt is coming from by check the PCNT.int_st.val 
 *              when it is coming from PCNT_UNIT0, the value will be 2^0 which equal to 1. same applies
 *              when it is coming from another PCNT_UNIT, e.g PCNT_UNIT_1 will produce value equal to (2^1) = 2
 *              just before this handler finish, it reset the PCNT interrupt status by set the PCNT.int_clr.val to 1
 *              To reset the PCNT_UNIT_0, set the PCNT.int_clr.val to (2^0) = 1
 *              To reset the PCNT_UNIT_1, set the PCNT.int_clr.val to (2^1) = 2
 *              ...
 *              ...
 *              To reset the PCNT_UNIT_7, set the PCNT.int_clr.val to (2^7) = 256
*/
static void IRAM_ATTR pcnt_intr_handler(void *arg)
{
    portENTER_CRITICAL_ISR(&timerMux_1); // key to lock shared resource (variable), so the value doesn't changed by another program
    unsigned long currentMillis = millis(); // Time at instant ISR was called
    uint32_t intr_status = PCNT.int_st.val; // get the pcnt unit interrupt, PCNT_UNIT_0 = 1, PCNT_UNIT_1 = 2, .... PCNT_UNIT_7 = 256
    for (size_t i = 0; i < 8; i++)
    {
      energyMeter[i].update(intr_status);
    }    
    PCNT.int_clr.val = intr_status; // reset the pcnt interrupt, set the corellated bit to 1 to reset
    portEXIT_CRITICAL_ISR(&timerMux_1); // unlock the key
}

/**
 * @brief 1 second timer interrupt
*/
void IRAM_ATTR onTimer()
{
  // return;
  mode++;
  if(mode >= 2)
  {
    mode = 0;
  }
  for (size_t i = 0; i < 8; i++)
  {
    energyMeter[i].pause();
  }
  
  for (size_t i = 0; i < 8; i++)
  {
    energyMeter[i].calculate();
    energyMeter[i].clear();
    energyMeter[i].setMode(mode);
    energyMeter[i].resume();
  }
  
  isTriggered = true;

}


/**
 * @brief initialize Pulse Counter PCNT
 *        @note this function will configure PCNT and register an event when counter reached high limit, the "pcnt_intr_handler" ISR registered as handler for this event
 * 
 * @param inputPin  input pulse signal pin
 * @param lowLimit  low limit value to trigger an event
 * @param hihgLimit high limit value to trigger an event
 * @param unit      PCNT unit
 *                  @note ESP32 has 8 PCNT unit start from PCNT_UNIT_0 - PCNT_UNIT_8, refer to "driver/pcnt.h" for more info
 * @param channel   PCNT channel
 *                  @note each PCNT unit has 2 channel which are PCNT_CHANNEL_0 & PCNT_CHANNEL_1, refer to "driver/pcnt.h" for more info 
*/
void initPcnt(int inputPin, int16_t lowLimit, int16_t highLimit, pcnt_unit_t unit, pcnt_channel_t channel)
{
  pcnt_config_t pcnt_config = {
    .pulse_gpio_num = inputPin,    // set gpio for pulse input gpio
    .ctrl_gpio_num = -1,            // no gpio for control
    .lctrl_mode = PCNT_MODE_KEEP,   // when control signal is low, keep the primary counter mode
    .hctrl_mode = PCNT_MODE_KEEP,   // when control signal is high, keep the primary counter mode
    .pos_mode = PCNT_COUNT_INC,     // increment the counter on positive edge
    .neg_mode = PCNT_COUNT_DIS,     // do nothing on falling edge
    .counter_h_lim = highLimit,     // set the higlimit counter
    .counter_l_lim = 0,             // set the lowlimit counter
    .unit = unit,               /*!< PCNT unit number */
    .channel = channel
  }; 
  pcnt_unit_config(&pcnt_config); // pass the config parameter of pcnt
  // Enable the PCNT interrupt
  pcnt_event_enable(unit, PCNT_EVT_H_LIM); // register the event callback when counter reached the maximum counter limit according to pcnt_config

  pcnt_counter_pause(unit); // pause the counter of pcnt unit
  pcnt_counter_clear(unit); // clear the counter of pcnt unit

  pcnt_isr_register(pcnt_intr_handler, NULL, 0, NULL); // register isr handler, when the event is triggered, the isr will be executed
  pcnt_intr_enable(unit); // enable the interrupt of pcnt
  // Start the PCNT counter
  pcnt_counter_resume(unit); // resume the pcnt counting pulse
}

void set_clock_gpio(int freqPin, int channel)
{
  ledcSetup(channel, 10, 7); // set resolution to 7 bit (2 ** 7 = 128), 1 bit represent (1 / 128) * 100% = 0.7%
  ledcAttachPin(freqPin, channel);
  ledcWrite(channel, 124); // set high duration to (124 / 128) * 100% = 96.875%
}

/**
 * @brief function to produce a clock pulse using ledc driver
 *        @note this function produce 50% duty cycle clock pulse
 * @param freqPin output pin to produce the pulse signal
 * @param channel ledc has 16 channel start from 0 - 15
 *        @note channel 0 & 1 share same timer clock
 *              channel 2 & 3 share same timer clock
 *              channel 4 & 5 share same timer clock
 *              ...
 *              ...
 *              channel 14 & 15 share same timer clock
 *              pin with same timer clock will produce pulse with same frequency
 * @param frequency frequency to set, unit in Hertz
*/
void set_clock_gpio_hf(int freqPin, int channel, int frequency)
{
  ledcSetup(channel, frequency, 1); // set resolution to 1 bit (2^1 = 2), 1 bit represent (1 / 2) * 100% = 50%
  ledcAttachPin(freqPin, channel);
  ledcWrite(channel, 1); // set high duration to (1 / 2) * 100% = 50%
}


void WiFiGotIP(WiFiEvent_t event, WiFiEventInfo_t info){
  digitalWrite(internalLed, HIGH);
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.print("Subnet Mask: ");
  Serial.println(WiFi.subnetMask());
  Serial.print("Gateway IP: ");
  Serial.println(WiFi.gatewayIP());
  Serial.print("DNS 1: ");
  Serial.println(WiFi.dnsIP(0));
  Serial.print("DNS 2: ");
  Serial.println(WiFi.dnsIP(1));
  Serial.print("Hostname: ");
  Serial.println(WiFi.getHostname());
  MBbridge.start(port, 4, 3000, 1);
}

void WiFiStationConnected(WiFiEvent_t event, WiFiEventInfo_t info){
  Serial.println("Wifi Connected");
}

void WiFiStationDisconnected(WiFiEvent_t event, WiFiEventInfo_t info){
  digitalWrite(internalLed, LOW);
  Serial.println("Disconnected from WiFi access point");
  Serial.print("WiFi lost connection. Reason: ");
  Serial.println(info.wifi_sta_disconnected.reason);
}

String createJsonResponse() {
  StaticJsonDocument<1024> doc;

  JsonArray pulse_data = doc.createNestedArray("data");
  for (size_t i = 0; i < PCNT_UNIT_MAX; i++)
  {
    JsonObject pulse_data_0 = pulse_data.createNestedObject();
    EnergyMeterData data = energyMeter[i].getEnergyMeterData();
    pulse_data_0["unit"] = data.unit;
    pulse_data_0["frequency"] = data.frequency;
    pulse_data_0["voltage"] = data.voltage;
    pulse_data_0["current"] = data.current;
    pulse_data_0["power"] = data.power;
  }
  String output;
  serializeJson(doc, output);
  return output;
}

void fillData(DTSU666_Data dtsuData[], ModbusMessage response, TypeToken typeToken)
{
  uint8_t serverId = response.getServerID();
  uint8_t index = serverId -1;
  uint8_t data[4];
  uint8_t offset = 3;
  uint16_t dat;
  int32_t low;
  int32_t high;
  switch (typeToken)
  {
  case TypeToken::REQUEST_VERSION:
    dtsuData[index].id = serverId;
    offset = response.get(offset, dtsuData[index].rev);
    offset = response.get(offset, dtsuData[index].ucode);
    offset = response.get(offset, dtsuData[index].clre);
    offset = response.get(offset, dtsuData[index].net);
    break;
  case TypeToken::REQUEST_RATE:
    dtsuData[index].id = serverId;
    offset = response.get(offset, dtsuData[index].irAt);
    offset = response.get(offset, dtsuData[index].urAt);
    break;
  case TypeToken::REQUEST_DISPLAY:
    dtsuData[index].id = serverId;
    offset = response.get(offset, dtsuData[index].disp);
    offset = response.get(offset, dtsuData[index].blcd);
    offset = response.get(offset, dtsuData[index].endian);
    break;
  case TypeToken::REQUEST_PROTOCOL:
    dtsuData[index].id = serverId;
    offset = response.get(offset, dtsuData[index].protocol);
    offset = response.get(offset, dtsuData[index].baud);
    offset = response.get(offset, dtsuData[index].addr);
    break;
  case TypeToken::REQUEST_DATA:
    dtsuData[index].id = serverId;
    offset = response.get(offset, dtsuData[index].Uab);
    offset = response.get(offset, dtsuData[index].Ubc);
    offset = response.get(offset, dtsuData[index].Uca);
    offset = response.get(offset, dtsuData[index].Ua);
    offset = response.get(offset, dtsuData[index].Ub);
    offset = response.get(offset, dtsuData[index].Uc);
    offset = response.get(offset, dtsuData[index].Ia);
    offset = response.get(offset, dtsuData[index].Ib);
    offset = response.get(offset, dtsuData[index].Ic);
    offset = response.get(offset, dtsuData[index].Pt);
    offset = response.get(offset, dtsuData[index].Pa);
    offset = response.get(offset, dtsuData[index].Pb);
    offset = response.get(offset, dtsuData[index].Pc);
    break;
  case TypeToken::REQUEST_PF:
    dtsuData[index].id = serverId;
    offset = response.get(offset, dtsuData[index].Pft);
    offset = response.get(offset, dtsuData[index].Pfa);
    offset = response.get(offset, dtsuData[index].Pfb);
    offset = response.get(offset, dtsuData[index].Pfc);
    break;
  case TypeToken::REQUEST_FREQ:
    dtsuData[index].id = serverId;
    offset = response.get(offset, dtsuData[index].Freq);
    break;
  case TypeToken::REQUEST_IMPORT:
    dtsuData[index].id = serverId;
    offset = response.get(offset, dtsuData[index].ImpEp);
    break;
  case TypeToken::REQUEST_EXPORT:
    dtsuData[index].id = serverId;
    offset = response.get(offset, dtsuData[index].ExpEp);
    break;
  default:
    break;
  }
}

// Serial interrupt handler function
void IRAM_ATTR serialInterrupt() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  // Read incoming data from Serial port
  // char receivedChar = Serial.read();
  bool receivedChar = true;
  // Send the received data to the Serial task
  xQueueSendFromISR(serialQueueHandle, &receivedChar, &xHigherPriorityTaskWoken);
  xQueueSendFromISR(serialQueueHandle, &receivedChar, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void handleData(ModbusMessage response, uint32_t token)
{
  Serial.printf("Response: serverID=%d, FC=%d, Token=%08X, length=%d:\n", response.getServerID(), response.getFunctionCode(), token, response.size());
  for (auto& byte : response) {
    Serial.printf("%02X ", byte);
  }
  TypeToken t = static_cast<TypeToken>(token);
  fillData(dtsu666Data, response, t);
  // Serial.println("");
}

void handleError(Error error, uint32_t token) 
{
  // ModbusError wraps the error code and provides a readable error message for it
  // ModbusError me(error);
  // Serial.println("Modbus Error");
  // Serial.printf("Error response: %02X - %s\n", (int)me, (const char *)me);
}

void sendTask(void* parameter)
{
  while (1)
  {
    if(isReady)
    {
      uint8_t receivedFlag;
      if (xSemaphoreTake(myLock, portMAX_DELAY) == pdTRUE)
      {
        if (writeCommand.size() > 0)
        {
          Serial.println("Command buffer before send : " + String(writeCommand.size()));
          Serial.println(writeCommand.at(0));
          writeCommand.remove(0);
          Serial.println("Command buffer after send : " + String(writeCommand.size()));
        }
        else
        {
          // Serial.println("scheduler");
        }
        
        
        // Serial.println("===========Begin=========");
        // for (size_t i = 0; i < 8; i++)
        // {
        //   EnergyMeterData *ptr;
        //   ptr = jsonData[i].dataPointer;
        //   Serial.println("Display : " + String(i));
        //   Serial.println("Counter : " + String(jsonData[i].counter));
        //   Serial.println("ID : " + String(jsonData[i].id));
        //   Serial.println("Device Name : " + String(jsonData[i].deviceName));
        //   for (size_t j = 0; j < jsonData[i].energyMeterDataSize; j++)
        //   {
        //     Serial.println("Unit : " + String((ptr+j)->unit));
        //     Serial.println("Frequency : " + String((ptr+j)->frequency));
        //     Serial.println("Voltage : " + String((ptr+j)->voltage));
        //     Serial.println("Current : " + String((ptr+j)->current));
        //     Serial.println("Power : " + String((ptr+j)->power));
        //   }
        // }
        // Serial.println("==========End===========");
      }
      xSemaphoreGive(myLock);
      // if(xQueueReceive(serialQueueHandle, &receivedFlag, portMAX_DELAY) == pdTRUE)
      // {
      //   // Serial.println("Blink Task");
      //   digitalWrite(internalLed, LOW);
      //   vTaskDelay(pdMS_TO_TICKS(500));
      //   digitalWrite(internalLed, HIGH);
      //   vTaskDelay(pdMS_TO_TICKS(500));
      // } 
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void serialTask(void* parameter)
{
  while (1) 
  {
    // Wait for data in the Serial queue
    uint8_t receivedData;
    // Serial.println("Serial Scheduler");
    // bool receivedData;
    // UBaseType_t stackHighWaterMark = uxTaskGetStackHighWaterMark(sendTaskHandle);
    // Serial.println(stackHighWaterMark);
    
    if (xQueueReceive(serialQueueHandle, &receivedData, portMAX_DELAY) == pdTRUE) 
    {
      // Process the received data
      // ...
      if (xSemaphoreTake(myLock, portMAX_DELAY) == pdTRUE)
      {
        while (Serial.available())
        {
          char data = Serial.read();
          if(data == '\n')
          {
            if(serialHandler.parse(dataString, dataPack, &Serial))
            {
              // Serial.println("Data : " + String(number));
              // String s = createJsonResponse();
              // Serial.println(s);
              Serial.println("======Data Received========");
              
            }
            else
            {
              Serial.println("======Data Error========");
            }
            number++;
            dataString = "";
          }
          else
          {
            dataString += data;
          }
          // Serial.print(data);
        }
        xSemaphoreGive(myLock);        
      }
    }  
  }
}

void test1()
{
  static unsigned long next_request = millis();
  // Error err = MB.addRequest(TypeToken::REQUEST_DATA, 1, READ_HOLD_REGISTER, 0x2000, 26);
  // Shall we do another request?
  if (millis() - next_request > READ_INTERVAL) 
  {
    // Yes.
    data_ready = false;
    // Issue the request
    Error err = MB.addRequest(TypeToken::REQUEST_DATA, 1, READ_HOLD_REGISTER, 0x2000, 26);
    if (err!=SUCCESS) {
      ModbusError e(err);
      // LOG_E("Error creating request: %02X - %s\n", (int)e, (const char *)e);
    }
    // Save current time to check for next cycle
    next_request = millis();
  } else {
    // // No, but we may have another response
    // if (data_ready) {
    //   // We do. Print out the data
    //   Serial.printf("Requested at %8.3fs:\n", request_time / 1000.0);
    //   for (uint8_t i = 0; i < NUM_VALUES; ++i) {
    //     Serial.printf("   %04X: %8.3f\n", i * 2 + FIRST_REGISTER, values[i]);
    //   }
    //   Serial.printf("----------\n\n");
    //   data_ready = false;
    // }
  }
}

void modbusTask(void* parameter)
{
  int token = 1200;
  int slaveId = 1;
  while (1)
  {
    if(isReady)
    {
      if (token > 1208)
      {
        token = 1200;
        slaveId++;
      }
      if(slaveId > 1)
      {
        slaveId = 1;
      }
      TypeToken t = static_cast<TypeToken>(token);
      int registerAddress;
      int numberRegister;
      switch (t)
      {
      case TypeToken::REQUEST_VERSION :
        registerAddress = 0x0;
        numberRegister = 4;
        break;
      case TypeToken::REQUEST_RATE :
        registerAddress = 0x6;
        numberRegister = 2;
        break;
      case TypeToken::REQUEST_DISPLAY :
        registerAddress = 0xA;
        numberRegister = 3;
        break;
      case TypeToken::REQUEST_PROTOCOL :
        registerAddress = 0x2C;
        numberRegister = 3;
        break;
      case TypeToken::REQUEST_DATA :
        registerAddress = 0x2000;
        numberRegister = 26;
        break;
      case TypeToken::REQUEST_PF :
        registerAddress = 0x202A;
        numberRegister = 8;
        break;
      case TypeToken::REQUEST_FREQ :
        registerAddress = 0x2044;
        numberRegister = 2;
        break;
      case TypeToken::REQUEST_IMPORT :
        registerAddress = 0x101E;
        numberRegister = 2 ;
        break;
      case TypeToken::REQUEST_EXPORT :
        registerAddress = 0x1028;
        numberRegister = 2;
        break;
      default:
        registerAddress = 0x2000;
        numberRegister = 26;
        break;
      }
      
      // if(modbusRequest.size() > 0)
      // {
      //   ModbusRequest mr = modbusRequest.front();
      //   uint8_t count = mr.registerCount*2;
        
      //   // uint16_t wData[] = { 0x1111, 0x2222, 0x3333, 0x4444, 0x5555, 0x6666 };

      //   // Error err = MB.addRequest(1300, 1, WRITE_MULT_REGISTERS, 33, 6, 12, wData);
      //   Error err = MB.addRequest(mr.token, mr.slaveId, mr.fc, mr.registerLocation, mr.registerCount, count, mr.modbusData);
      //   if (err!=SUCCESS) {
      //     ModbusError e(err);
      //     Serial.println((const char*) e);
      //     // LOG_E("Error creating request: %02X - %s\n", (int)e, (const char *)e);
      //   }
      //   else
      //   {
      //     modbusRequest.remove(0);
      //   }
      // }
      // else
      // {
      //   Error err = MB.addRequest(t, slaveId, READ_HOLD_REGISTER, registerAddress, numberRegister);
      //   if (err!=SUCCESS) {
      //     ModbusError e(err);
      //     // Serial.println((const char*) e);
      //     // LOG_E("Error creating request: %02X - %s\n", (int)e, (const char *)e);
      //   }
      //   else
      //   {
      //     token++;
      //   }
      // }      
    }
    vTaskDelay(pdMS_TO_TICKS(200));
  }
  
}

/**
 * @brief assign value to energyData
*/
void initEnergyMeterData()
{
  for (size_t i = 0; i < 64; i++)
  {
    energyData[i].unit = i;
    energyData[i].frequency = i*10000;
    energyData[i].voltage = i*1000;
    energyData[i].current = i*100;
    energyData[i].power = i*10;
    // for (size_t j = 0; j < 8; j++)
    // {
    //   int startAddr = i * 8;
    //   energyData[startAddr+j].unit = j;
    //   energyData[startAddr+j].frequency = j*10000;
    //   energyData[startAddr+j].voltage = j*1000;
    //   energyData[startAddr+j].current = j*100;
    //   energyData[startAddr+j].power = j*10;
    // }
  }
}

void initRelayStatus()
{
  for (size_t i = 0; i < 8; i++)
  {
    relayStatus[i].line_status.val = 1 << i;
  }
  
}

/**
 * @brief assign value to jsonData
 *        @note jsonData is struct to use for http request
*/
void initJsonData()
{
  EnergyMeterData *startPtr;
  RelayStatus *startRelayStatusPtr;
  startPtr = energyData;
  startRelayStatusPtr = relayStatus;
  for (size_t i = 0; i < 8; i++)
  {
    startPtr = energyData;
    jsonData[i].counter = 0;
    jsonData[i].id = i;
    jsonData[i].deviceName = "device_name_" + String(i);
    jsonData[i].energyMeterDataSize = 8;
    jsonData[i].dataPointer = startPtr + (i*8);
    jsonData[i].relayStatusPointer = startRelayStatusPtr + i;
    jsonData[i].relayStatusDataSize = 12;
  }
  
}

/**
 * @brief assign value to jsonHeader
 *        @note jsonHeader is header for http request
*/
void initJsonHeader()
{
  jsonHeader.deviceSn = "device_sn";
  jsonHeader.ip = "0.0.0.0";
  jsonHeader.type = "sdbctrl";
}

void initDataPack()
{
  dataPack.jsonData = jsonData;
  dataPack.relayStatus = relayStatus;
}

void initDtsu666Data()
{
  for (size_t i = 0; i < 8; i++)
  {
    dtsu666Data[i].id = i+1;
    dtsu666Data[i].deviceName = "dtsu666_" + String(i+1);
    // dtsu666Data[i].ImpEp = i*100000;
    // dtsu666Data[i].ExpEp = i*500000;
  }
  
}

void setup() {
  // put your setup code here, to run once:
  initEnergyMeterData();
  initJsonData();
  initJsonHeader();
  initDataPack();
  initRelayStatus();
  initDtsu666Data();
  FastLED.addLeds<WS2812, ledPin, GRB>(leds, numberOfLed);
  FastLED.setBrightness(20);
  // pinMode(cfPin, INPUT_PULLDOWN);
  // pinMode(cf1Pin, INPUT_PULLDOWN);
  // pinMode(selPin, OUTPUT);
  // attachInterrupt(GPIO_NUM_3, serialInterrupt, FALLING);
  pinMode(internalLed, OUTPUT);
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 1000000, true);
  
  Serial.setRxBufferSize(2048);
  Serial.begin(115200);
  Serial.setRxTimeout(1);

  MB.onDataHandler(&handleData);
  MB.onErrorHandler(&handleError);
  MB.setTimeout(100);
  RTUutils::prepareHardwareSerial(Serial2);
  Serial2.setRxBufferSize(1024);
  Serial2.begin(9600);
  MB.begin(Serial2, 1);

  for (size_t i = 1; i < 247; i++)
  {
      MBbridge.attachServer(i, i, ANY_FUNCTION_CODE, &MB);
  }
  
  MBbridge.listServer();
  
  

  // serialQueueHandle = xQueueCreate(queueSize, sizeof(uint8_t));

  // Create the serial task
  // xTaskCreate(serialTask, "SerialTask", stackSize, NULL, 1, &serialTaskHandle);
  // xTaskCreate(sendTask, "SendTask", stackSize, NULL, 1, &sendTaskHandle);
  xTaskCreate(modbusTask, "modbusTask", stackSize, NULL, 1, &modbusTaskHandle);
  // Serial.println("Stopping Task");
  // vTaskSuspendAll();
  // Set up the interrupt handler
  // Serial.onReceive(serialInterrupt);

  WiFi.disconnect(true);
  WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, INADDR_NONE);
  WiFi.mode(WIFI_MODE_NULL);
  delay(100);
  WiFi.setHostname(hostName.c_str());
  WiFi.mode(WIFI_STA);
  WiFi.onEvent(WiFiStationConnected, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_CONNECTED);
  WiFi.onEvent(WiFiGotIP, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_GOT_IP);

  WiFi.begin(ssid, password);
  int timeout = 0;
  while(timeout < 10)
  {
    if (WiFi.status() != WL_CONNECTED)
    {
        delay(100);
        timeout++;
    }
    else
    {
        break;
    }
  }

  if (timeout < 10)
  {
    digitalWrite(internalLed, HIGH);
  }
  else
  {
    digitalWrite(internalLed, LOW);
  }

  server.on("/get-data-all", HTTP_GET, [](AsyncWebServerRequest *request)
  {
    Serial.println("GET Line Data");
    request->send(200, "application/json", createJsonResponse());
  });

  server.on("/get-data", HTTP_GET, [](AsyncWebServerRequest *request)
  {
    Serial.println("GET Line Data");
    String buffer;
    size_t jsonDataSize = sizeof(jsonData) / sizeof(jsonData[0]);
    if (jsonHandler.httpBuildData(request, jsonHeader, jsonData, jsonDataSize, buffer) > 0)
    {
      request->send(200, "application/json", buffer);
    }
    else
    {
      request->send(400);
    }
  });

  /**
   * @brief TO DO LIST
  */
  // AsyncCallbackJsonWebHandler *setRelayHttpHandler = new AsyncCallbackJsonWebHandler("/set-relay", [](AsyncWebServerRequest *request, JsonVariant &json)
  // {
  //     String input = json.as<String>();
  //     String output;
  //     if (jsonHandler.httpRelayWrite(input.c_str(), output))
  //     {
  //       if(writeCommand.size() == writeCommand.max_size())
  //       {
  //         writeCommand.remove(0);
  //       }
  //       writeCommand.push_back(output);
  //       request->send(200, "application/json", jsonHandler.httpResponseOk());
  //     }
  //     else
  //     {
  //       request->send(400);
  //     }
  // });

  AsyncCallbackJsonWebHandler *setRelayHandler = new AsyncCallbackJsonWebHandler("/set-relay", [](AsyncWebServerRequest *request, JsonVariant &json)
  {
    Serial.println("POST Request Line Data");
    String response;
    String input = json.as<String>();
    if (webServerHandler.processRelayRequest(input.c_str(), response, commandList) > 0)
    {    
      request->send(200, "application/json", response);
    }
    else
    {
      request->send(400);
    }
    });

  AsyncCallbackJsonWebHandler *setModbusHandler = new AsyncCallbackJsonWebHandler("/write-dtsu666", [](AsyncWebServerRequest *request, JsonVariant &json)
  {
    Serial.println("POST Modbus Write");
    String response;
    String input = json.as<String>();
    ModbusRequest mr;
    if (jsonHandler.httpModbusWrite(input.c_str(),mr, response))
    {    
      Serial.println("ID : " + String(mr.slaveId));
      Serial.println("FC : " + String(mr.fc));
      Serial.println("Location : " + String(mr.registerLocation));
      Serial.println("Number : " + String(mr.registerCount));
      modbusRequest.push_back(mr);
      request->send(200, "application/json", response);
    }
    else
    {
      request->send(400);
    }
    });

  server.on("/get-relay-status", HTTP_GET, [](AsyncWebServerRequest *request)
  {
    Serial.println("GET Relay Status");
    String buffer;
    size_t jsonDataSize = sizeof(jsonData) / sizeof(jsonData[0]);
    if (jsonHandler.httpBuildRelayStatus(request, jsonHeader, jsonData, jsonDataSize, buffer) > 0)
    {
      request->send(200, "application/json", buffer);
    }
    else
    {
      request->send(400);
    }
  });

  server.on("/get-dtsu666-data", HTTP_GET, [](AsyncWebServerRequest *request)
  {
    Serial.println("GET DTSU666 Data");
    String buffer;
    size_t dtsu666DataSize = sizeof(dtsu666Data) / sizeof(dtsu666Data[0]);
    if (jsonHandler.httpDtsu666Data(request, jsonHeader, dtsu666Data, dtsu666DataSize, buffer) > 0)
    {
      request->send(200, "application/json", buffer);
    }
    else
    {
      request->send(400);
    }
  });


  // server.addHandler(setRelayHttpHandler);
  server.addHandler(setRelayHandler);
  server.addHandler(setModbusHandler);
  server.begin();

  // initPcnt(cfPin, 0, highLimit, PCNT_UNIT_0, PCNT_CHANNEL_0); // pulse counter for active power
  // initPcnt(cf1Pin, 0, highLimit, PCNT_UNIT_1, PCNT_CHANNEL_0); // pulse counter for voltage or current rms


  /**
   * init pcnt unit and channel
  */
  // initPcnt(pcntPin[0], 0, highLimit, PCNT_UNIT_0, PCNT_CHANNEL_0);
  // initPcnt(pcntPin[1], 0, highLimit, PCNT_UNIT_1, PCNT_CHANNEL_0);
  // initPcnt(pcntPin[2], 0, highLimit, PCNT_UNIT_2, PCNT_CHANNEL_0);
  // initPcnt(pcntPin[3], 0, highLimit, PCNT_UNIT_3, PCNT_CHANNEL_0);
  // initPcnt(pcntPin[4], 0, highLimit, PCNT_UNIT_4, PCNT_CHANNEL_0);
  // initPcnt(pcntPin[5], 0, highLimit, PCNT_UNIT_5, PCNT_CHANNEL_0);
  // initPcnt(pcntPin[6], 0, highLimit, PCNT_UNIT_6, PCNT_CHANNEL_0);
  // initPcnt(pcntPin[7], 0, highLimit, PCNT_UNIT_7, PCNT_CHANNEL_0);

  for (size_t i = 0; i < 8; i++)
  {
    energyMeter[i].registerHandler(pcnt_intr_handler);
    energyMeter[i].setMode(0);
    energyMeter[i].setCalibrator(coef_v, coef_i, coef_p);
    // if (i%2)
    // {
    //   energyMeter[i].setMode(3);
    // }
  }

  for (size_t i = 0; i < 8; i++)
  {
    energyMeter[i].start();
  }
  
  
  /**
   * set ledc output
  */
  // set_clock_gpio_hf(ledcPin[0], 0, 100000);
  // set_clock_gpio_hf(ledcPin[1], 2, 200000);
  // set_clock_gpio_hf(ledcPin[2], 4, 300000);
  // set_clock_gpio_hf(ledcPin[3], 6, 400000);
  // set_clock_gpio_hf(ledcPin[4], 8, 500000);
  // set_clock_gpio_hf(ledcPin[5], 10, 600000);
  // set_clock_gpio_hf(ledcPin[6], 12, 700000);
  // set_clock_gpio_hf(ledcPin[7], 14, 800000);

  // pinMode(ledcPin[7], OUTPUT);

  // set_clock_gpio_hf(freqPin_1, 0, 1000);
  // delay(100);
  // set_clock_gpio_hf(freqPin_2, 2, 500);
  // delay(100);
  timerAlarmEnable(timer); // enable 1 second timer
  // Serial.println("Resuming Task");
  // xTaskResumeAll();
  isReady = true;
}

void loop() {
  // Serial.println("TEST");
  // UBaseType_t stackWatermark = uxTaskGetStackHighWaterMark(modbusTaskHandle);
  // Serial.print("Stack Watermark: ");
  // Serial.println(stackWatermark);
  if ((WiFi.status() != WL_CONNECTED) && (millis() - lastReconnectMillis >= reconnectInterval)) {
    Serial.println("WiFi Disconnected");    
    digitalWrite(internalLed, LOW);
    Serial.println("Reconnecting to WiFi...");
    WiFi.disconnect();
    WiFi.reconnect();
    lastReconnectMillis = millis();
  }
  
  /**
   * @brief relay shift register process
  */
  if(commandList.size() > 0)
  {
    Command c = commandList.front();
    Serial.println("==================");
    switch (c.type)
    {
      case RELAY:
        Serial.println("Command Type : RELAY");
        for(int i = 0; i < c.relayData.number; i++)
        {
          int pin = relayControl.write(c.relayData.lineList[i], c.relayData.valueList[i]);
          Serial.println("Pin : " + String(pin));
          sr.setNoUpdate(pin, HIGH);
          ledControl.write(c.relayData.lineList[i], c.relayData.valueList[i], leds);
        }
        sr.updateRegisters();
        // Serial.println("Set To High");
        delay(20);
        for(int i = 0; i < c.relayData.number; i++)
        {
          int pin = relayControl.write(c.relayData.lineList[i], c.relayData.valueList[i]);
          sr.setNoUpdate(pin, LOW);
        }
        sr.updateRegisters();
        delay(20);
        // Serial.println("Set To Low");
        FastLED.show();
      break;
    }
    // Serial.println("Line : " + String(c.line));
    // Serial.println("Value : " + String(c.value));    
    commandList.remove(0);
  }
  // relayControl.write(1, true);
  // ledControl.write(1, true, leds);
  // delay(1000);
  // relayControl.write(1, false);
  // ledControl.write(1, false, leds);
  // if(isTriggered)
  // {
  //   // if (mode >= 2)
  //   // {
  //   //   mode = 0;
  //   // }
  //   Serial.println("==========Frequency Measurement===========");
  //   Serial.println("Measurement : " + String(number));
    
  //   for (size_t i = 0; i < 8; i++)
  //   {
  //     EnergyMeterData data = energyMeter[i].getEnergyMeterData();
  //     // int energyMeterMode = energyMeter[i].getMode();
  //     // switch (energyMeterMode)
  //     // {
  //     //   case 0:
  //     //       Serial.println("Current Mode");
  //     //     break;
  //     //   case 1:
  //     //       Serial.println("Voltage Mode");
  //     //     break;
  //     //   default:
  //     //       Serial.println("Power Mode");
  //     // }
  //     Serial.println("Frequency " + String(data.unit) + " : " + String(data.frequency) + " Hz");
  //     Serial.println("Voltage " + String(data.unit) + " : " + String(data.voltage) + " mV");
  //     Serial.println("Current " + String(data.unit) + " : " + String(data.current) + " mA");
  //     Serial.println("Power " + String(data.unit) + " : " + String(data.power) + " mW");
  //     energyMeter[i].setMode(mode);
  //   }

  //   Serial.println("==================End=====================");
  //   isTriggered = false;
  //   number++;
  // }
}