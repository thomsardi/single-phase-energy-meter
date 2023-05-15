#ifndef WEB_SERVER_HANDLER_H
#define WEB_SERVER_HANDLER_H

#include <Arduino.h>
#include <ArduinoJson.h>
#include <ESPAsyncWebServer.h>
#include <DataDef.h>
#include <Vector.h>

class WebServerHandler {
    public:
        WebServerHandler();
        int processLineDataRequest(AsyncWebServerRequest *request, String &buffer, const LineData lineData[], size_t arrSize);
        int processRelayRequest(const String &input, String &buffer, Vector<Command> &commandList);
        void parser(const String &input, char delimiter, Vector<String> &valueVec);
        ~WebServerHandler();
    private:
        bool isNumber(const String &input);
};

#endif