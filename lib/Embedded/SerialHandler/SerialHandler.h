#ifndef SERIALHANDLER_H
#define SERIALHANDLER_H

// #define SERIALHANDLER_DEBUG

// #ifdef SERIALHANDLER_DEBUG
//     #define LOG_PRINT(x)    Serial.print(x)
//     #define LOG_PRINTLN(x)  Serial.println(x)
// #else
//     #define LOG_PRINT(x)
//     #define LOG_PRINTLN(x)
// #endif

#include <Arduino.h>
#include <ArduinoJson.h>
#include <Vector.h>
#include "SerialData.h"
#include <DataDef.h>
// #include <JsonHandler.h>

class SerialHandler {
    public :
        // SerialHandler(Stream *serial);
        SerialHandler();
        bool parse(String input, DataPack &dataPack, Stream *serial);
        bool write(const WriteCommand &writeCommand, Stream *serial);
        String createCommand(const WriteCommand &writeCommand, JsonObjectConst data);
        String buildDataRequest();

    private :
        bool dataReadResponseParser(String input, DataPack &dataPack, Stream *serial);
        bool lineWriteResponseParser(String input);
        void merge(JsonObject dest, JsonObjectConst src);
};


#endif