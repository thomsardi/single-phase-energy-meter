#ifndef SERIALDATA_H
#define SERIALDATA_H

#include <Arduino.h>
#include <EnergyMeter.h>
#include <JsonData.h>

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
    RELAY_DATA = 2
};

struct RelayStatus {
    union {
        struct {
            uint16_t line0: 1;           /*This is the raw bit for line0 status.*/
            uint16_t line1: 1;           /*This is the raw bit for line1 status.*/
            uint16_t line2: 1;           /*This is the raw bit for line2 status.*/
            uint16_t line3: 1;           /*This is the raw bit for line3 status.*/
            uint16_t line4: 1;           /*This is the raw bit for line4 status.*/
            uint16_t line5: 1;           /*This is the raw bit for line5 status.*/
            uint16_t line6: 1;           /*This is the raw bit for line6 status.*/
            uint16_t line7: 1;           /*This is the raw bit for line7 status.*/
            uint16_t line8: 1;           /*This is the raw bit for line8 status.*/
            uint16_t line9: 1;           /*This is the raw bit for line9 status.*/
            uint16_t line10: 1;          /*This is the raw bit for line10 status.*/
            uint16_t line11: 1;          /*This is the raw bit for line11 status.*/
            uint16_t line12: 1;          /*This is the raw bit for line12 status.*/
            uint16_t reserved: 4;
        };
        int16_t val;
    } line_status; 
};

struct DataPack {
    JsonData *jsonData;
    RelayStatus *relayStatus;
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