#ifndef SERIALDATA_H
#define SERIALDATA_H

#include <Arduino.h>
// #include <EnergyMeter.h>

enum FunctionCode
{
    READCOIL = 1,
    READ_DISCRETE_INPUT = 2,
    READ_HOLDING_REGISTER = 3,
    READ_INPUT_REGISTER = 4,
    WRITE_SINGLE_COIL = 5,
    WRITE_SINGLE_HOLDING_REGISTER = 6,
    WRITE_MULTIPLE_COILS = 15,
    WRITE_MULTIPLE_HOLDING_REGISTER = 16
};

enum RegisterList
{
    ENERGY_DATA = 1,
    RELAY_DATA = 2,
    MASTER_DEVICE_SN = 3,
    SLAVE_DEVICE_SN = 4
};

struct RelayData {
    int line;
    int state;
};

struct WriteData {
    size_t dataSize;
    union {
        RelayData *relayData;
    } data;
};

struct WriteCommand {
    uint id;
    uint functionCode;
    uint registerLocation;
    String writedata;
};

#endif