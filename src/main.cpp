#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <HTTPClient.h>
#include <AsyncJson.h>
// #include "driver/ledc.h"
#include "driver/pcnt.h"
#include "soc/pcnt_struct.h"

unsigned int number = 0;
float coef_u = 0.125; // coefficient for voltage measurement, to get this value use the following formula coef u = voltage on voltmeter / reading frequency
int ledcPin[8] = {4, 13, 14, 15, 16, 17, 18, 19};
int pcntPin[8] = {21, 22, 23, 25, 26, 27, 32, 33};
int freqPin_1 = 21;
int freqPin_2 = 22;
int freqPin_3 = 23;
int hfFreq = 1000000;
int hfFreq2 = 2000000;
int cfPin = 19; // pin for active power
int cf1Pin = 18; // pin for voltage or current RMS depending on selPin
int selPin = 23; // pin to switch measurement between voltage or current RMS
bool isChanged = 0;
int16_t highLimit = 30000;

hw_timer_t *timer = NULL;

volatile bool isTriggered, isTriggered_2;
uint8_t volatile tick;
volatile unsigned long capturedTime;
unsigned long lastTime;
volatile long interval;
int readingFrequency[8];
volatile int pulse;
volatile int mult[8];
volatile int overflow;
volatile int overflow_2;
int statusCheck, statusCheck2;

portMUX_TYPE timerMux_1 = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE timerMux_2 = portMUX_INITIALIZER_UNLOCKED;

xQueueHandle pcnt_evt_queue;   // A queue to handle pulse counter events
pcnt_isr_handle_t user_isr_handle = NULL; //user's ISR service handle


const int internalLed = 2; //internal led pin

const char *ssid = "RnD_Sundaya";
const char *password = "sundaya22";
const String hostName = "RnD-BL-Board";

unsigned long lastReconnectMillis = 0;
int reconnectInterval = 5000;

AsyncWebServer server(80);

/* A sample structure to pass events from the PCNT
 * interrupt handler to the main program.
 */
typedef struct {
    int unit;  // the PCNT unit that originated an interrupt
    uint32_t status; // information on the event type that caused the interrupt
    unsigned long timeStamp; // The time the event occured
} pcnt_evt_t;

// Define the ISR function to be called when the PCNT event occurs
void IRAM_ATTR pcnt_isr_handler(void* arg) {
  portENTER_CRITICAL_ISR(&timerMux_1);
  overflow++;
  isTriggered = true;
  uint32_t status = PCNT.int_st.val;
  // PCNT.int_clr.val = BIT(PCNT_UNIT_0);
  PCNT.int_clr.val = status;
  statusCheck = status;
  portEXIT_CRITICAL_ISR(&timerMux_1);
}

/* Decode what PCNT's unit originated an interrupt
 * and pass this information together with the event type
 * and timestamp to the main program using a queue.
 */
static void IRAM_ATTR pcnt_intr_handler(void *arg)
{
    portENTER_CRITICAL_ISR(&timerMux_1);
    unsigned long currentMillis = millis(); //Time at instant ISR was called
    uint32_t intr_status = PCNT.int_st.val;
    int i = 0;
    pcnt_evt_t evt;
    // portBASE_TYPE HPTaskAwoken = pdFALSE;
    statusCheck = intr_status;
    for (i = 0; i < PCNT_UNIT_MAX; i++) {
        if (intr_status & (BIT(i))) {
            mult[i]++;
        }
    }
    PCNT.int_clr.val = intr_status;
    // overflow++;
    // for (i = 0; i < PCNT_UNIT_MAX; i++) {
    //     if (intr_status & (BIT(i))) {
    //         evt.unit = i;
    //         /* Save the PCNT event type that caused an interrupt
    //            to pass it to the main program */
    //         evt.status = PCNT.status_unit[i].val;
    //         evt.timeStamp = currentMillis; 
    //         PCNT.int_clr.val = BIT(i);
    //         xQueueSendFromISR(pcnt_evt_queue, &evt, &HPTaskAwoken);
    //         if (HPTaskAwoken == pdTRUE) {
    //             portYIELD_FROM_ISR();
    //         }
    //     }
    // }
    portEXIT_CRITICAL_ISR(&timerMux_1);
}

void IRAM_ATTR onTimer()
{
  pcnt_unit_t pcnt_unit[8] = {
    PCNT_UNIT_0,
    PCNT_UNIT_1,
    PCNT_UNIT_2,
    PCNT_UNIT_3,
    PCNT_UNIT_4,
    PCNT_UNIT_5,
    PCNT_UNIT_6,
    PCNT_UNIT_7,
  };
  for (size_t i = 0; i < PCNT_UNIT_MAX; i++)
  {
    /* code */
    pcnt_counter_pause(pcnt_unit[i]);
  }
  
  for (size_t i = 0; i < PCNT_UNIT_MAX; i++)
  {
    int16_t pulse = 0;
    pcnt_get_counter_value(pcnt_unit[i], &pulse);
    readingFrequency[i] = mult[i] * highLimit + pulse;
    mult[i] = 0;
    pcnt_counter_clear(pcnt_unit[i]);
    
  }
  isTriggered = true;
  // digitalWrite(selPin, !digitalRead(selPin));
  for (size_t i = 0; i < PCNT_UNIT_MAX; i++)
  {
    /* code */
    pcnt_counter_resume(pcnt_unit[i]);
  }
  
}

void IRAM_ATTR intHandler()
{
  // capturedTime = micros();
  // interval = capturedTime - lastTime;
  // lastTime = capturedTime;
  // readingFrequency = 1000000 / interval;
  if(pulse > 20000)
  {
    overflow++;
    pulse = 0;
  }
  pulse++;
}

void set_high_speed_counter(int pin, int16_t highLimit, pcnt_unit_t unit, pcnt_channel_t channel, void (*fn)(void *))
{
  pcnt_config_t pcnt_config = {
    .pulse_gpio_num = pin,    // set gpio for pulse input gpio
    .ctrl_gpio_num = -1,            // no gpio for control
    .lctrl_mode = PCNT_MODE_KEEP,   // when control signal is low, keep the primary counter mode
    .hctrl_mode = PCNT_MODE_KEEP,   // when control signal is high, keep the primary counter mode
    .pos_mode = PCNT_COUNT_INC,     // increment the counter on positive edge
    .neg_mode = PCNT_COUNT_DIS,     // do nothing on falling edge
    .counter_h_lim = highLimit,
    .counter_l_lim = 0,
    .unit = unit,               /*!< PCNT unit number */
    .channel = channel
  }; 
  pcnt_unit_config(&pcnt_config);
  // Enable the PCNT interrupt
  pcnt_event_enable(unit, PCNT_EVT_H_LIM);

  pcnt_counter_pause(unit);
  pcnt_counter_clear(unit);

  pcnt_isr_register(fn, NULL, 0, NULL);
  pcnt_intr_enable(unit);
  // Start the PCNT counter
  pcnt_counter_resume(unit);

}

void set_pcnt0(int pin, int16_t highLimit)
{
  pcnt_config_t pcnt_config = {
    .pulse_gpio_num = pin,    // set gpio for pulse input gpio
    .ctrl_gpio_num = -1,            // no gpio for control
    .lctrl_mode = PCNT_MODE_KEEP,   // when control signal is low, keep the primary counter mode
    .hctrl_mode = PCNT_MODE_KEEP,   // when control signal is high, keep the primary counter mode
    .pos_mode = PCNT_COUNT_INC,     // increment the counter on positive edge
    .neg_mode = PCNT_COUNT_DIS,     // do nothing on falling edge
    .counter_h_lim = highLimit,
    .counter_l_lim = 0,
    .unit = PCNT_UNIT_1,               /*!< PCNT unit number */
    .channel = PCNT_CHANNEL_0
  }; 
  pcnt_unit_config(&pcnt_config);
  // Enable the PCNT interrupt
  pcnt_event_enable(PCNT_UNIT_1, PCNT_EVT_H_LIM);

  pcnt_counter_pause(PCNT_UNIT_1);
  pcnt_counter_clear(PCNT_UNIT_1);

  pcnt_isr_register(pcnt_intr_handler, NULL, 0, NULL);
  pcnt_intr_enable(PCNT_UNIT_1);
  // Start the PCNT counter
  pcnt_counter_resume(PCNT_UNIT_1);

}

void set_pcnt1(int pin, int16_t highLimit)
{
  pcnt_config_t pcnt_config = {
    .pulse_gpio_num = pin,    // set gpio for pulse input gpio
    .ctrl_gpio_num = -1,            // no gpio for control
    .lctrl_mode = PCNT_MODE_KEEP,   // when control signal is low, keep the primary counter mode
    .hctrl_mode = PCNT_MODE_KEEP,   // when control signal is high, keep the primary counter mode
    .pos_mode = PCNT_COUNT_INC,     // increment the counter on positive edge
    .neg_mode = PCNT_COUNT_DIS,     // do nothing on falling edge
    .counter_h_lim = highLimit,
    .counter_l_lim = 0,
    .unit = PCNT_UNIT_2,               /*!< PCNT unit number */
    .channel = PCNT_CHANNEL_0
  }; 
  pcnt_unit_config(&pcnt_config);
  // Enable the PCNT interrupt
  pcnt_event_enable(PCNT_UNIT_2, PCNT_EVT_H_LIM);

  pcnt_counter_pause(PCNT_UNIT_2);
  pcnt_counter_clear(PCNT_UNIT_2);

  pcnt_isr_register(pcnt_intr_handler, NULL, 0, NULL);
  pcnt_intr_enable(PCNT_UNIT_2);
  // Start the PCNT counter
  pcnt_counter_resume(PCNT_UNIT_2);

}

void initPcnt(int inputPin, int16_t lowLimit, int16_t highLimit, pcnt_unit_t unit, pcnt_channel_t channel)
{
  pcnt_config_t pcnt_config = {
    .pulse_gpio_num = inputPin,    // set gpio for pulse input gpio
    .ctrl_gpio_num = -1,            // no gpio for control
    .lctrl_mode = PCNT_MODE_KEEP,   // when control signal is low, keep the primary counter mode
    .hctrl_mode = PCNT_MODE_KEEP,   // when control signal is high, keep the primary counter mode
    .pos_mode = PCNT_COUNT_INC,     // increment the counter on positive edge
    .neg_mode = PCNT_COUNT_DIS,     // do nothing on falling edge
    .counter_h_lim = highLimit,
    .counter_l_lim = 0,
    .unit = unit,               /*!< PCNT unit number */
    .channel = channel
  }; 
  pcnt_unit_config(&pcnt_config);
  // Enable the PCNT interrupt
  pcnt_event_enable(unit, PCNT_EVT_H_LIM);

  pcnt_counter_pause(unit);
  pcnt_counter_clear(unit);

  pcnt_isr_register(pcnt_intr_handler, NULL, 0, NULL);
  pcnt_intr_enable(unit);
  // Start the PCNT counter
  pcnt_counter_resume(unit);
}

void set_clock_gpio(int freqPin, int channel)
{
  ledcSetup(channel, 10, 7); // set resolution to 7 bit (2 ** 7 = 128), 1 bit represent (1 / 128) * 100% = 0.7%
  ledcAttachPin(freqPin, channel);
  ledcWrite(channel, 124); // set high duration to (124 / 128) * 100% = 96.875%
}

void set_clock_gpio_hf(int freqPin, int channel, int frequency)
{
  ledcSetup(channel, frequency, 1); // set resolution to 7 bit (2 ** 7 = 128), 1 bit represent (1 / 128) * 100% = 0.7%
  ledcAttachPin(freqPin, channel);
  ledcWrite(channel, 1); // set high duration to (124 / 128) * 100% = 96.875%
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
  StaticJsonDocument<512> doc;

  JsonArray pulse_data = doc.createNestedArray("pulse_data");
  for (size_t i = 0; i < PCNT_UNIT_MAX; i++)
  {
    JsonObject pulse_data_0 = pulse_data.createNestedObject();
    pulse_data_0["pcnt"] = i+1;
    pulse_data_0["frequency"] = readingFrequency[i];
  }
  String output;
  serializeJson(doc, output);
  return output;
}

void setup() {
  // put your setup code here, to run once:
  // pinMode(cfPin, INPUT_PULLDOWN);
  // pinMode(cf1Pin, INPUT_PULLDOWN);
  // pinMode(selPin, OUTPUT);
  pinMode(internalLed, OUTPUT);
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 1000000, true);
  // attachInterrupt(interruptPin, intHandler, RISING);
  Serial.begin(115200);
  while (!Serial);

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

  server.on("/get-data", HTTP_GET, [](AsyncWebServerRequest *request)
  {
    Serial.println("GET Line Data");
    request->send(200, "application/json", createJsonResponse());
  });

  server.begin();

  // initPcnt(cfPin, 0, highLimit, PCNT_UNIT_0, PCNT_CHANNEL_0); // pulse counter for active power
  // initPcnt(cf1Pin, 0, highLimit, PCNT_UNIT_1, PCNT_CHANNEL_0); // pulse counter for voltage or current rms

  initPcnt(pcntPin[0], 0, highLimit, PCNT_UNIT_0, PCNT_CHANNEL_0);
  initPcnt(pcntPin[1], 0, highLimit, PCNT_UNIT_1, PCNT_CHANNEL_0);
  initPcnt(pcntPin[2], 0, highLimit, PCNT_UNIT_2, PCNT_CHANNEL_0);
  initPcnt(pcntPin[3], 0, highLimit, PCNT_UNIT_3, PCNT_CHANNEL_0);
  initPcnt(pcntPin[4], 0, highLimit, PCNT_UNIT_4, PCNT_CHANNEL_0);
  initPcnt(pcntPin[5], 0, highLimit, PCNT_UNIT_5, PCNT_CHANNEL_0);
  initPcnt(pcntPin[6], 0, highLimit, PCNT_UNIT_6, PCNT_CHANNEL_0);
  initPcnt(pcntPin[7], 0, highLimit, PCNT_UNIT_7, PCNT_CHANNEL_0);

  set_clock_gpio_hf(ledcPin[0], 0, 100000);
  set_clock_gpio_hf(ledcPin[1], 2, 200000);
  set_clock_gpio_hf(ledcPin[2], 4, 300000);
  set_clock_gpio_hf(ledcPin[3], 6, 400000);
  set_clock_gpio_hf(ledcPin[4], 8, 500000);
  set_clock_gpio_hf(ledcPin[5], 10, 600000);
  set_clock_gpio_hf(ledcPin[6], 12, 700000);
  set_clock_gpio_hf(ledcPin[7], 14, 800000);

  // set_clock_gpio_hf(freqPin_1, 0, 1000);
  // delay(100);
  // set_clock_gpio_hf(freqPin_2, 2, 500);
  // delay(100);
  timerAlarmEnable(timer);

}

void loop() {
  // Serial.println("TEST");
  if ((WiFi.status() != WL_CONNECTED) && (millis() - lastReconnectMillis >= reconnectInterval)) {
    Serial.println("WiFi Disconnected");    
    digitalWrite(internalLed, LOW);
    Serial.println("Reconnecting to WiFi...");
    WiFi.disconnect();
    WiFi.reconnect();
    lastReconnectMillis = millis();
  }
  
  if(isTriggered)
  {
    Serial.println("==========Frequency Measurement===========");
    Serial.println("Measurement : " + String(number));
    for (size_t i = 0; i < PCNT_UNIT_MAX; i++)
    {
      /* code */
      int voltage = static_cast<int>(readingFrequency[i]*coef_u);
      Serial.println("Frequency " + String(i) + " : " + String(readingFrequency[i]) + " Hz");
      Serial.println("Voltage Reading " + String(i) + " : " + String(voltage) + " V");
      // readingFrequency[i] = 0;
    }
    Serial.println("==================End=====================");
    isTriggered = false;
    number++;
  }
    

}