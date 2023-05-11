#ifndef JSONDATA_H
#define JSONDATA_H

#include <Arduino.h>
#include <EnergyMeterData.h>

struct JsonHeader
{
    String deviceSn;
    String ip;
    String type;

};

struct JsonData
{
    uint counter;
    uint8_t id;
    String deviceName;
    EnergyMeterData *dataPointer;
    size_t energyMeterDataSize;
};

#endif