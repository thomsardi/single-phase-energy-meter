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
float coef_i = 0.013;
float coef_p = 2.54;
int ledcPin[8] = {4, 13, 14, 15, 16, 17, 18, 19};
int pcntPin[8] = {21, 22, 23, 25, 26, 27, 32, 33};
int freqPin_1 = 21;
int freqPin_2 = 22;
int freqPin_3 = 23;
int hfFreq = 1000000;
int hfFreq2 = 2000000;
int cfPin = 19; // pin for active power value
int cf1Pin = 18; // pin for voltage or current RMS depending on selPin
int selPin = 23; // pin to switch measurement between voltage or current RMS (0 for current measurement, 1 for voltage measurement)
bool isChanged = 0;
int16_t highLimit = 30000; // value for counter overflow

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
    int mult;
    int readingFrequency;
} pcnt_evt_t;

pcnt_evt_t evt[8];

/**
 * @brief ISR handler when PCNT event trigger
 *        @note first, this handler will decode which interrupt is coming from by check the PCNT.int_st.val 
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
    statusCheck = intr_status;
    for (int i = 0; i < PCNT_UNIT_MAX; i++) {
        if (intr_status & (BIT(i))) // find the pcnt unit, BIT(i) = 1 << i, if bit AND result is 1 then the pcnt unit is found
        {
            evt[i].unit = i;
            evt[i].mult++; // increment the multiplier
        }
    }
    PCNT.int_clr.val = intr_status; // reset the pcnt interrupt, set the corellated bit to 1 to reset
    portEXIT_CRITICAL_ISR(&timerMux_1); // unlock the key
}

/**
 * @brief ISR handler for timer interrupt
 *        @note this function is an handler for timer interrupt, this will convert the PCNT pulse into frequency, and store the information into pcnt_evt_t struct
*/
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
    pcnt_counter_pause(pcnt_unit[i]); // pause the pcnt counter, so the values doesnt get updated
  }
  
  for (size_t i = 0; i < PCNT_UNIT_MAX; i++)
  {
    int16_t pulse = 0;
    pcnt_get_counter_value(pcnt_unit[i], &pulse);
    evt[i].readingFrequency = evt[i].mult * highLimit + pulse; // calculate the frequency, mult is multiplier (mult +1 for every 30000 pulse) then add the remainder of pulse
    evt[i].mult = 0; // reset the multipier to 0
    pcnt_counter_clear(pcnt_unit[i]); // clear the pcnt counter (set the counter to 0)
    
  }
  isTriggered = true;
  // digitalWrite(selPin, !digitalRead(selPin));
  for (size_t i = 0; i < PCNT_UNIT_MAX; i++)
  {
    pcnt_counter_resume(pcnt_unit[i]); // resume pcnt pulse counting
  }
  
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
    pulse_data_0["pcnt"] = evt[i].unit;
    pulse_data_0["frequency"] = evt[i].readingFrequency;
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


  /**
   * init pcnt unit and channel
  */
  initPcnt(pcntPin[0], 0, highLimit, PCNT_UNIT_0, PCNT_CHANNEL_0);
  initPcnt(pcntPin[1], 0, highLimit, PCNT_UNIT_1, PCNT_CHANNEL_0);
  initPcnt(pcntPin[2], 0, highLimit, PCNT_UNIT_2, PCNT_CHANNEL_0);
  initPcnt(pcntPin[3], 0, highLimit, PCNT_UNIT_3, PCNT_CHANNEL_0);
  initPcnt(pcntPin[4], 0, highLimit, PCNT_UNIT_4, PCNT_CHANNEL_0);
  initPcnt(pcntPin[5], 0, highLimit, PCNT_UNIT_5, PCNT_CHANNEL_0);
  initPcnt(pcntPin[6], 0, highLimit, PCNT_UNIT_6, PCNT_CHANNEL_0);
  initPcnt(pcntPin[7], 0, highLimit, PCNT_UNIT_7, PCNT_CHANNEL_0);

  /**
   * set ledc output
  */
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
  timerAlarmEnable(timer); // enable 1 second timer

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
      int voltage = static_cast<int>(evt[i].readingFrequency*1000*coef_u); // unit in milli
      Serial.println("Frequency " + String(i) + " : " + String(evt[i].readingFrequency) + " Hz");
      Serial.println("Voltage Reading " + String(i) + " : " + String(voltage) + " V");
      // readingFrequency[i] = 0;
    }
    Serial.println("==================End=====================");
    isTriggered = false;
    number++;
  }
    

}