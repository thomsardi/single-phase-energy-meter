#ifndef DATA_DEF_H
#define DATA_DEF_H

#include <Arduino.h>
#include <SerialData.h>
#include <JsonData.h>
#include <EnergyMeterData.h>

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

struct JsonData
{
    uint counter;
    uint8_t id;
    String deviceName;
    EnergyMeterData *dataPointer;
    RelayStatus *relayStatusPointer;
    size_t energyMeterDataSize;
    size_t relayStatusDataSize;
    size_t dtsu666DataSize;
};

struct DTSU666_Data{
    uint counter;
    uint8_t id;
    String deviceName;
    float Uab, Ubc, Uca;
    float Ua, Ub, Uc;
    float Ia, Ib, Ic;
    float Pt;
    float Pa, Pb, Pc;
    float Pft;
    float Pfa, Pfb, Pfc;
    float Freq;
    float ImpEp;
    float ExpEp;
};

struct DataPack {
    JsonData *jsonData;
    RelayStatus *relayStatus;
    DTSU666_Data *dtsu666Data;
};

/**
 * @brief used for relay control
*/
enum CommandType{
    RELAY = 1,
};

struct LineData
{
    int voltage;
    int current;
    int power;
};

struct RelayData
{
    int number;
    int lineList[36];
    int valueList[36];
};

struct Command 
{
    int type;
    RelayData relayData;
};

enum TypeToken {
    REQUEST_DATA = 1200,
    REQUEST_PF = 1201,
    REQUEST_FREQ = 1202,
    REQUEST_IMPORT = 1203,
    REQUEST_EXPORT = 1204
};

#endif