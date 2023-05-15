#include "WebServerHandler.h"

WebServerHandler::WebServerHandler()
{

}

WebServerHandler::~WebServerHandler()
{

}

int WebServerHandler::processLineDataRequest(AsyncWebServerRequest *request, String &buffer, const LineData lineData[], size_t arrSize)
{
    StaticJsonDocument<1024> doc;
    int line = 0;
    int voltage = 0;
    
    if(!request->hasParam("line"))
    {
        return -1;
    }
    String input = request->getParam("line")->value();
    Serial.println(input);
    String value[12];
    Vector<String> valueVec;
    valueVec.setStorage(value);
    parser(input, ',', valueVec);
    JsonArray data = doc.createNestedArray("line-data");
    
    for (auto &temp : valueVec)
    {
        if(isNumber(temp))
        {
            line = temp.toInt();
            if (line > 64)
            {
                return -1;
            }
            JsonObject data_0 = data.createNestedObject();
            data_0["line"] = line;
            data_0["voltage"] = lineData[line].voltage;
            data_0["current"] = lineData[line].current;
            data_0["power"] = lineData[line].power;
        }
        else
        {
            return -1;
        }
    }
    
    serializeJson(doc, buffer);

    return 1;
}

int WebServerHandler::processRelayRequest(const String &input, String &buffer, Vector<Command> &commandList)
{
    StaticJsonDocument<1024> doc;

    DeserializationError error = deserializeJson(doc, input);

    if (error) {
        Serial.print("deserializeJson() failed: ");
        Serial.println(error.c_str());
        return -1;
    }

    if(!doc.containsKey("data"))
    {
        return -1;
    }

    Command command;

    command.type = RELAY;

    int number = 0;
    for (JsonObject data_item : doc["data"].as<JsonArray>()) 
    {
        command.relayData.lineList[number] = data_item["line"]; // 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12
        command.relayData.valueList[number] = data_item["value"]; // 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1
        number++;
    }

    if (number <= 0)
    {
        return -1;
    }

    command.relayData.number = number;
    number = 0;

    commandList.push_back(command);

    StaticJsonDocument<16> docResponse;

    docResponse["status"] = 1;

    serializeJson(docResponse, buffer);

    return 1;
}


bool WebServerHandler::isNumber(const String &input)
{
    for (char const &ch : input) {
        if (std::isdigit(ch) == 0) 
            return false;
    }
    return true;
}

void WebServerHandler::parser(const String &input, char delimiter, Vector<String> &valueVec)
{
    int index = 0;
    int lastIndex = 0;
    int i = 0;
    bool isContinue = true;
    while(isContinue)
    {
        if (i >= valueVec.max_size())
        {
            break;
        }
        lastIndex = input.indexOf(delimiter, index);
        String value = input.substring(index, lastIndex);
        valueVec.push_back(value);
        i++;
        if (lastIndex <= 0)
        {
            isContinue = false;
        }
        index = lastIndex+1;
    }
}