#ifndef SERIALHANDLER_H
#define SERIALHANDLER_H

// #define ENERGYMETER_DEBUG

#ifdef SERIALHANDLER_DEBUG
    #define LOG_PRINT(x)    Serial.print(x)
    #define LOG_PRINTLN(x)  Serial.println(x)
#else
    #define LOG_PRINT(x)
    #define LOG_PRINTLN(x)
#endif

#include <Arduino.h>
#include <ArduinoJson.h>
#include <Vector.h>
#include "JsonData.h"

class SerialHandler {
    public :
        SerialHandler();
        int parse(String input, JsonData jsonData[]);

        String buildDataRequest();

    private :
        bool dataResponseParser(String input, JsonData jsonData[]);
};


#endif