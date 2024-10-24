#ifndef JSONHANDLER_H
#define JSONHANDLER_H

#define JSONHANDLER_DEBUG

#ifdef JSONHANDLER_DEBUG
    #define LOG_PRINT(x)    Serial.print(x)
    #define LOG_PRINTLN(x)  Serial.println(x)
#else
    #define LOG_PRINT(x)
    #define LOG_PRINTLN(x)
#endif

#include <Arduino.h>
#include "JsonData.h"
#include <SerialHandler.h>
#include <SerialData.h>
#include <ArduinoJson.h>
#include <Vector.h>
#include <ESPAsyncWebServer.h>

class JsonHandler {
    public :
        JsonHandler(Stream *serial);
        int httpBuildData(AsyncWebServerRequest *request, const JsonHeader &jsonHeader, const JsonData jsonData[], size_t jsonDataSize, String &buffer);
        bool httpDtsu666Data(AsyncWebServerRequest *request, const JsonHeader &jsonHeader, const DTSU666_Data dtsu666Data[], size_t dtsu666DataSize, String &buffer);
        bool httpBuildRelayStatus(AsyncWebServerRequest *request, const JsonHeader &jsonHeader, const JsonData jsonData[], size_t jsonDataSize, String &buffer);
        bool httpRelayWrite(const char* input, String &output);
        bool httpModbusWrite(const char* input, ModbusRequest &modbusRequest, String &output);
        String httpResponseOk();
        static void merge(JsonObject dest, JsonObjectConst src);
    private :
        void parser(const String &input, char delimiter, Vector<String> &valueVec);
        bool isNumber(const String &input);
        Stream *_serial;
        uint16_t _modbusData[32];
};


#endif