#include "SerialHandler.h"

SerialHandler::SerialHandler()
{

}

int SerialHandler::parse(String input, JsonData jsonData[])
{
    if (dataResponseParser(input, jsonData))
    {
        return 1;
    }
    return -1;
}

bool SerialHandler::dataResponseParser(String input, JsonData jsonData[])
{
    StaticJsonDocument<1024> doc;

    DeserializationError error = deserializeJson(doc, input);

    if (error) {
    Serial.print("deserializeJson() failed: ");
    Serial.println(error.c_str());
    return;
    }

    int id = doc["id"]; // 103
    int index = id - 1;
    jsonData[index].id = id; 
    // const char* device_name = doc["device_name"]; // "device_name_3"
    jsonData[index].deviceName = doc["device_name"].as<String>();
    EnergyMeterData *ptr = jsonData[index].dataPointer;
    for (JsonObject device_data_item : doc["device_data"].as<JsonArray>()) {

        (ptr+index)->unit = device_data_item["unit"]; // 24, 25, 26, 27, 28, 29, 30, 31
        long device_data_item_frequency = device_data_item["frequency"]; // 240000, 250000, 260000, 270000, ...
        int device_data_item_voltage = device_data_item["voltage"]; // 24000, 25000, 26000, 27000, 28000, 29000, ...
        int device_data_item_current = device_data_item["current"]; // 2400, 2500, 2600, 2700, 2800, 2900, 3000, ...
        int device_data_item_power = device_data_item["power"]; // 240, 250, 260, 270, 280, 290, 300, 310

    }
}
