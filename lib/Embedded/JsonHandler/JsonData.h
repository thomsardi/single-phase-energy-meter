#ifndef JSONDATA_H
#define JSONDATA_H

#include <Arduino.h>
#include <EnergyMeterData.h>
#include <DataDef.h>

struct JsonHeader
{
    String deviceSn;
    String ip;
    String type;

};

#endif