#include "JsonHandler.h"

JsonHandler::JsonHandler(Stream *serial)
{
    _serial = serial;
}

int JsonHandler::httpBuildData(AsyncWebServerRequest *request, const JsonHeader &jsonHeader, const JsonData jsonData[], size_t jsonDataSize, String &buffer)
{
    if (!request->hasParam("id"))
    {
        return -1;
    }

    DynamicJsonDocument doc(8192);
    String value[12];
    Vector <String> valueVec;
    valueVec.setStorage(value);
    EnergyMeterData *dataPointer;

    String input = request->getParam("id")->value();
    parser(input, ',', valueVec);
    
    doc["device_sn"] = jsonHeader.deviceSn;
    doc["ip"] = jsonHeader.ip;
    doc["type"] = jsonHeader.type;
    JsonArray data = doc.createNestedArray("data");
    for (auto &temp : valueVec)
    {
        JsonObject data_0 = data.createNestedObject();
        if (isNumber(temp))
        {
            int id = temp.toInt();
            for (int i = 0; i < jsonDataSize; i++)
            {
                if (id == jsonData[i].id)
                {
                    data_0["counter"] = jsonData[i].counter;
                    data_0["id"] = jsonData[i].id;
                    data_0["device_name"] = jsonData[i].deviceName;
                    dataPointer = jsonData[i].dataPointer;
                    JsonArray data_0_device_data = data_0.createNestedArray("device_data");
                    for (int j = 0; j < jsonData[i].energyMeterDataSize; j++)
                    {
                        JsonObject data_0_device_data_0 = data_0_device_data.createNestedObject();
                        data_0_device_data_0["unit"] = (dataPointer+j)->unit;
                        data_0_device_data_0["frequency"] = (dataPointer+j)->frequency;;
                        data_0_device_data_0["voltage"] = (dataPointer+j)->voltage;
                        data_0_device_data_0["current"] = (dataPointer+j)->current;
                        data_0_device_data_0["power"] = (dataPointer+j)->power;
                    } 
                }
            }
        }        
    }
    serializeJson(doc, buffer);
    return 1;
}

bool JsonHandler::httpBuildRelayStatus(AsyncWebServerRequest *request, const JsonHeader &jsonHeader, const JsonData jsonData[], size_t jsonDataSize, String &buffer)
{
    if (!request->hasParam("id"))
    {
        return 0;
    }

    DynamicJsonDocument doc(8192);
    String value[12];
    Vector <String> valueVec;
    valueVec.setStorage(value);
    RelayStatus *dataPointer;

    String input = request->getParam("id")->value();
    parser(input, ',', valueVec);
    
    doc["device_sn"] = jsonHeader.deviceSn;
    doc["ip"] = jsonHeader.ip;
    doc["type"] = jsonHeader.type;
    JsonArray data = doc.createNestedArray("data");

    for (auto &temp : valueVec)
    {
        JsonObject data_0 = data.createNestedObject();
        if (isNumber(temp))
        {
            int id = temp.toInt();
            for (int i = 0; i < jsonDataSize; i++)
            {
                if (id == jsonData[i].id)
                {
                    data_0["counter"] = jsonData[i].counter;
                    data_0["id"] = jsonData[i].id;
                    data_0["device_name"] = jsonData[i].deviceName;
                    dataPointer = jsonData[i].relayStatusPointer;
                    JsonArray data_0_device_data = data_0.createNestedArray("device_data");
                    for (int j = 0; j < jsonData[i].relayStatusDataSize; j++)
                    {
                        JsonObject data_0_device_data_0 = data_0_device_data.createNestedObject();
                        data_0_device_data_0["line"] = j;
                        int16_t state = ((dataPointer->line_status.val) >> j) & 1;
                        data_0_device_data_0["state"] = state;
                    } 
                }
            }
        }        
    }
    serializeJson(doc, buffer);
    return 1;
}

bool JsonHandler::httpRelayWrite(const char* input, String &output)
{
    // String input;

    StaticJsonDocument<768> doc, doc2;

    DeserializationError error = deserializeJson(doc, input);

    if (error) {
        Serial.print("deserializeJson() failed: ");
        Serial.println(error.c_str());
        return 0;
    }

    if (!doc.containsKey("id"))
    {
        return 0;
    }

    if (!doc.containsKey("data"))
    {
        return 0;
    }
    WriteCommand w;
    w.id = doc["id"];
    w.functionCode = FunctionCode::WRITE_MULTIPLE_HOLDING_REGISTER;
    w.registerLocation = RegisterList::RELAY_DATA;
    JsonArray data = doc2.createNestedArray("data");
    for (JsonObject data_item : doc["data"].as<JsonArray>()) {

        JsonObject data_0 = data.createNestedObject();
        if (data_item.containsKey("line") && data_item.containsKey("state"))
        {
            data_0["line"] = data_item["line"]; // 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12
            data_0["state"] = data_item["state"]; // 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0
        }        
    }
    SerialHandler s;
    // serializeJson(doc2, output);
    // _serial->println(output);
    output = s.createCommand(w, doc2.as<JsonObject>());
    // _serial->println(output);
    return 1;
}

String JsonHandler::httpResponseOk()
{
    String output;
    StaticJsonDocument<16> doc;

    doc["status"] = 200;

    serializeJson(doc, output);
    return output;
}

void JsonHandler::parser(const String &input, char delimiter, Vector<String> &valueVec)
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

bool JsonHandler::isNumber(const String &input)
{
    for (char const &ch : input) {
        if (std::isdigit(ch) == 0) 
            return false;
    }
    return true;
}