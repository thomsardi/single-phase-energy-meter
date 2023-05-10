#include "JsonHandler.h"

JsonHandler::JsonHandler()
{

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