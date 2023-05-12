#include "SerialHandler.h"

// SerialHandler::SerialHandler(Stream *serial)
// {
//     // Serial = serial;
// }

SerialHandler::SerialHandler()
{
    // Serial = serial;
}

bool SerialHandler::parse(String input, DataPack &dataPack, Stream *serial)
{
    if (dataReadResponseParser(input, dataPack, serial))
    {
        return 1;
    }
    return 0;
}

bool SerialHandler::write(const WriteCommand &writeCommand, Stream *serial)
{
    StaticJsonDocument<768> doc;
    
    doc["data"] = writeCommand.writedata;
    String output;
    serializeJson(doc, output);
    serial->println(writeCommand.writedata);
    serial->println(output);
    return 1;
}

String SerialHandler::createCommand(const WriteCommand &writeCommand, JsonObjectConst data)
{
    StaticJsonDocument<1024> doc, doc2;
    String output;
    doc["id"] = writeCommand.id;
    doc["code"] = writeCommand.functionCode;
    doc["register"] = writeCommand.registerLocation;
    // doc2["test"] = 1000;
    merge(doc.as<JsonObject>(), data);
    serializeJson(doc, output);
    return output;
}

void SerialHandler::merge(JsonObject dest, JsonObjectConst src)
{
    for (auto kvp : src) 
    {
        dest[kvp.key()] = kvp.value();
    }
}

bool SerialHandler::dataReadResponseParser(String input, DataPack &dataPack, Stream *serial)
{
    StaticJsonDocument<1024> doc;

    DeserializationError error = deserializeJson(doc, input);
    if (error) {
        serial->print("deserializeJson() failed: ");
        serial->println(error.c_str());
        return 0;
    }

    int id = doc["id"]; // 1
    int index = id - 1;
    int code = doc["code"]; // 1
    int reg = doc["register"]; // 1

    if (code != FunctionCode::READ_HOLDING_REGISTER)
    {
        serial->println("Function Code Error");
        return 0;
    }

    if (reg != RegisterList::ENERGY_DATA)
    {
        serial->println("Register Error");
        return 0;
    }
    JsonData *jsonData = dataPack.jsonData;
    RelayStatus *relayStatus = dataPack.relayStatus;
    jsonData[index].id = id;
    jsonData[index].deviceName = doc["data"]["device_name"].as<String>();
    EnergyMeterData *ptr = jsonData[index].dataPointer;
    int cursor = 0;
    for (JsonObject data_device_data_item : doc["data"]["device_data"].as<JsonArray>()) {

        (ptr+cursor)->unit = data_device_data_item["unit"]; // 24, 25, 26, 27, 28, 29, 30, 31
        // serial->println("Unit : " + String((ptr+cursor)->unit));
        (ptr+cursor)->frequency = data_device_data_item["frequency"]; // 240000, 250000, 260000, 270000, ...
        // serial->println("Frequency : " + String((ptr+cursor)->frequency));
        (ptr+cursor)->voltage = data_device_data_item["voltage"]; // 24000, 25000, 26000, 27000, 28000, 29000, ...
        // serial->println("Voltage : " + String((ptr+cursor)->voltage));
        (ptr+cursor)->current = data_device_data_item["current"]; // 2400, 2500, 2600, 2700, 2800, 2900, 3000, ...
        // serial->println("Current : " + String((ptr+cursor)->current));
        (ptr+cursor)->power = data_device_data_item["power"]; // 240, 250, 260, 270, 280, 290, 300, 310
        // serial->println("Power : " + String((ptr+cursor)->power));
        cursor++;

    }
    jsonData[index].counter++;
    return 1;
}
