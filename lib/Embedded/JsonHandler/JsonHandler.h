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
#include <ESPAsyncWebServer.h>
#include "JsonData.h"
#include <SerialData.h>
#include <ArduinoJson.h>
#include <Vector.h>

class JsonHandler {
    public :
        JsonHandler();
        int httpBuildData(AsyncWebServerRequest *request, const JsonHeader &jsonHeader, const JsonData jsonData[], size_t jsonDataSize, String &buffer);
        bool httpRelayWrite(const char* input, WriteCommand &writeCommand);
        String httpResponseOk();
    private :
        void parser(const String &input, char delimiter, Vector<String> &valueVec);
        bool isNumber(const String &input);
};


#endif