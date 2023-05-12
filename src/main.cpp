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


HardwareSerial SerialPort(0);
JsonHandler jsonHandler(&SerialPort);
// SerialHandler serialHandler(&SerialPort);
SerialHandler serialHandler;


// Task handle and stack size
TaskHandle_t serialTaskHandle;
TaskHandle_t sendTaskHandle;
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

const char *ssid = "RnD_Sundaya";
const char *password = "sundaya22";
const String hostName = "RnD-BL-Board";

String dataString;

unsigned long lastReconnectMillis = 0;
int reconnectInterval = 5000;

AsyncWebServer server(80);

EnergyMeter energyMeter[8] = {
  EnergyMeter(pcntPin[0], ledcPin[2], 0, 0),
  EnergyMeter(pcntPin[1], 5, 1, 0),
  EnergyMeter(pcntPin[2], 5, 2, 0),
  EnergyMeter(pcntPin[3], 5, 3, 0),
  EnergyMeter(pcntPin[4], 5, 4, 0),
  EnergyMeter(pcntPin[5], 5, 5, 0),
  EnergyMeter(pcntPin[6], 5, 6, 0),
  EnergyMeter(pcntPin[7], 5, 7, 0)
};

JsonData jsonData[8];
JsonHeader jsonHeader;
DataPack dataPack;
RelayStatus relayStatus[8];
String writeCommandStorage[12];
Vector <String> writeCommand(writeCommandStorage);

EnergyMeterData energyData[64]; //divided into a group for every 8

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
  SerialPort.print("Connected to ");
  SerialPort.println(ssid);
  SerialPort.print("IP address: ");
  SerialPort.println(WiFi.localIP());
  SerialPort.print("Subnet Mask: ");
  SerialPort.println(WiFi.subnetMask());
  SerialPort.print("Gateway IP: ");
  SerialPort.println(WiFi.gatewayIP());
  SerialPort.print("DNS 1: ");
  SerialPort.println(WiFi.dnsIP(0));
  SerialPort.print("DNS 2: ");
  SerialPort.println(WiFi.dnsIP(1));
  SerialPort.print("Hostname: ");
  SerialPort.println(WiFi.getHostname());
}

void WiFiStationConnected(WiFiEvent_t event, WiFiEventInfo_t info){
  SerialPort.println("Wifi Connected");
}

void WiFiStationDisconnected(WiFiEvent_t event, WiFiEventInfo_t info){
  digitalWrite(internalLed, LOW);
  SerialPort.println("Disconnected from WiFi access point");
  SerialPort.print("WiFi lost connection. Reason: ");
  SerialPort.println(info.wifi_sta_disconnected.reason);
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

// SerialPort interrupt handler function
void IRAM_ATTR serialInterrupt() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  // Read incoming data from SerialPort port
  // char receivedChar = SerialPort.read();
  bool receivedChar = true;
  // Send the received data to the SerialPort task
  xQueueSendFromISR(serialQueueHandle, &receivedChar, &xHigherPriorityTaskWoken);
  xQueueSendFromISR(serialQueueHandle, &receivedChar, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
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
          SerialPort.println("Command buffer before send : " + String(writeCommand.size()));
          SerialPort.println(writeCommand.at(0));
          writeCommand.remove(0);
          SerialPort.println("Command buffer after send : " + String(writeCommand.size()));
        }
        else
        {
          // SerialPort.println("scheduler");
        }
        
        
        // SerialPort.println("===========Begin=========");
        // for (size_t i = 0; i < 8; i++)
        // {
        //   EnergyMeterData *ptr;
        //   ptr = jsonData[i].dataPointer;
        //   SerialPort.println("Display : " + String(i));
        //   SerialPort.println("Counter : " + String(jsonData[i].counter));
        //   SerialPort.println("ID : " + String(jsonData[i].id));
        //   SerialPort.println("Device Name : " + String(jsonData[i].deviceName));
        //   for (size_t j = 0; j < jsonData[i].energyMeterDataSize; j++)
        //   {
        //     SerialPort.println("Unit : " + String((ptr+j)->unit));
        //     SerialPort.println("Frequency : " + String((ptr+j)->frequency));
        //     SerialPort.println("Voltage : " + String((ptr+j)->voltage));
        //     SerialPort.println("Current : " + String((ptr+j)->current));
        //     SerialPort.println("Power : " + String((ptr+j)->power));
        //   }
        // }
        // SerialPort.println("==========End===========");
      }
      xSemaphoreGive(myLock);
      // if(xQueueReceive(serialQueueHandle, &receivedFlag, portMAX_DELAY) == pdTRUE)
      // {
      //   // SerialPort.println("Blink Task");
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
    // Wait for data in the SerialPort queue
    uint8_t receivedData;
    // SerialPort.println("Serial Scheduler");
    // bool receivedData;
    // UBaseType_t stackHighWaterMark = uxTaskGetStackHighWaterMark(sendTaskHandle);
    // SerialPort.println(stackHighWaterMark);
    
    if (xQueueReceive(serialQueueHandle, &receivedData, portMAX_DELAY) == pdTRUE) 
    {
      // Process the received data
      // ...
      if (xSemaphoreTake(myLock, portMAX_DELAY) == pdTRUE)
      {
        while (SerialPort.available())
        {
          char data = SerialPort.read();
          if(data == '\n')
          {
            if(serialHandler.parse(dataString, dataPack, &SerialPort))
            {
              // SerialPort.println("Data : " + String(number));
              // String s = createJsonResponse();
              // SerialPort.println(s);
              SerialPort.println("======Data Received========");
              
            }
            else
            {
              SerialPort.println("======Data Error========");
            }
            number++;
            dataString = "";
          }
          else
          {
            dataString += data;
          }
          // SerialPort.print(data);
        }
        xSemaphoreGive(myLock);        
      }
    }  
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

void setup() {
  // put your setup code here, to run once:
  initEnergyMeterData();
  initJsonData();
  initJsonHeader();
  initDataPack();
  initRelayStatus();
  // pinMode(cfPin, INPUT_PULLDOWN);
  // pinMode(cf1Pin, INPUT_PULLDOWN);
  // pinMode(selPin, OUTPUT);
  // attachInterrupt(GPIO_NUM_3, serialInterrupt, FALLING);
  pinMode(internalLed, OUTPUT);
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 1000000, true);

  SerialPort.setRxBufferSize(2048);
  SerialPort.begin(115200);
  SerialPort.setRxTimeout(1);

  serialQueueHandle = xQueueCreate(queueSize, sizeof(uint8_t));

  // Create the serial task
  xTaskCreate(serialTask, "SerialTask", stackSize, NULL, 1, &serialTaskHandle);
  xTaskCreate(sendTask, "SendTask", stackSize, NULL, 1, &sendTaskHandle);
  // SerialPort.println("Stopping Task");
  // vTaskSuspendAll();
  // Set up the interrupt handler
  SerialPort.onReceive(serialInterrupt);

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
    SerialPort.println("GET Line Data");
    request->send(200, "application/json", createJsonResponse());
  });

  server.on("/get-data", HTTP_GET, [](AsyncWebServerRequest *request)
  {
    SerialPort.println("GET Line Data");
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

  AsyncCallbackJsonWebHandler *setRelayHttpHandler = new AsyncCallbackJsonWebHandler("/set-relay", [](AsyncWebServerRequest *request, JsonVariant &json)
  {
      String input = json.as<String>();
      String output;
      if (jsonHandler.httpRelayWrite(input.c_str(), output))
      {
        if(writeCommand.size() == writeCommand.max_size())
        {
          writeCommand.remove(0);
        }
        writeCommand.push_back(output);
        request->send(200, "application/json", jsonHandler.httpResponseOk());
      }
      else
      {
        request->send(400);
      }
  });

  server.on("/get-relay-status", HTTP_GET, [](AsyncWebServerRequest *request)
  {
    SerialPort.println("GET Relay Status");
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

  server.addHandler(setRelayHttpHandler);
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
  set_clock_gpio_hf(ledcPin[0], 0, 100000);
  set_clock_gpio_hf(ledcPin[1], 2, 200000);
  // set_clock_gpio_hf(ledcPin[2], 4, 300000);
  set_clock_gpio_hf(ledcPin[3], 6, 400000);
  set_clock_gpio_hf(ledcPin[4], 8, 500000);
  set_clock_gpio_hf(ledcPin[5], 10, 600000);
  set_clock_gpio_hf(ledcPin[6], 12, 700000);
  set_clock_gpio_hf(ledcPin[7], 14, 800000);

  // pinMode(ledcPin[7], OUTPUT);

  // set_clock_gpio_hf(freqPin_1, 0, 1000);
  // delay(100);
  // set_clock_gpio_hf(freqPin_2, 2, 500);
  // delay(100);
  timerAlarmEnable(timer); // enable 1 second timer
  // SerialPort.println("Resuming Task");
  // xTaskResumeAll();
  isReady = true;
}

void loop() {
  // SerialPort.println("TEST");
  if ((WiFi.status() != WL_CONNECTED) && (millis() - lastReconnectMillis >= reconnectInterval)) {
    // SerialPort.println("WiFi Disconnected");    
    digitalWrite(internalLed, LOW);
    // SerialPort.println("Reconnecting to WiFi...");
    WiFi.disconnect();
    WiFi.reconnect();
    lastReconnectMillis = millis();
  }
  
  // if(isTriggered)
  // {
  //   SerialPort.println("==========Frequency Measurement===========");
  //   SerialPort.println("Measurement : " + String(number));
  //   for (size_t i = 0; i < PCNT_UNIT_MAX; i++)
  //   {
  //     /* code */
  //     int voltage = static_cast<int>(evt[i].readingFrequency*1000*coef_u); // unit in milli
  //     SerialPort.println("Frequency " + String(i) + " : " + String(evt[i].readingFrequency) + " Hz");
  //     SerialPort.println("Voltage Reading " + String(i) + " : " + String(voltage) + " V");
  //     // readingFrequency[i] = 0;
  //   }
  //   SerialPort.println("==================End=====================");
  //   isTriggered = false;
  //   number++;
  // }

  // if(isTriggered)
  // {
  //   if (mode >= 2)
  //   {
  //     mode = 0;
  //   }
  //   SerialPort.println("==========Frequency Measurement===========");
  //   SerialPort.println("Measurement : " + String(number));
    
  //   for (size_t i = 0; i < 8; i++)
  //   {
  //     EnergyMeterData data = energyMeter[i].getEnergyMeterData();
  //     // int energyMeterMode = energyMeter[i].getMode();
  //     // switch (energyMeterMode)
  //     // {
  //     //   case 0:
  //     //       SerialPort.println("Current Mode");
  //     //     break;
  //     //   case 1:
  //     //       SerialPort.println("Voltage Mode");
  //     //     break;
  //     //   default:
  //     //       SerialPort.println("Power Mode");
  //     // }
  //     SerialPort.println("Frequency " + String(data.unit) + " : " + String(data.frequency) + " Hz");
  //     SerialPort.println("Voltage " + String(data.unit) + " : " + String(data.voltage) + " mV");
  //     SerialPort.println("Current " + String(data.unit) + " : " + String(data.current) + " mA");
  //     SerialPort.println("Power " + String(data.unit) + " : " + String(data.power) + " mW");
  //     energyMeter[i].setMode(mode);
  //   }

  //   SerialPort.println("==================End=====================");
  //   isTriggered = false;
  //   number++;
  // }
}