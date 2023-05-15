#ifndef MODBUSRTU_DATA_H
#define MODBUSRTU_DATA_H

#include <Arduino.h>
#include <Vector.h>

namespace ModbusRTU
{
    struct ModbusRTU_Data
    {
        uint8_t slaveId;
        uint8_t functionCode;
    };

    enum FunctionCode
    {
        READCOIL = 1,
        READ_DISCRETE_INPUT = 2,
        READ_HOLDING_REGISTER = 3,
        READ_INPUT_REGISTER = 4,
        WRITE_SINGLE_COIL = 5,
        WRITE_SINGLE_HOLDING_REGISTER = 6,
        WRITE_MULTIPLE_COILS = 15,
    };

    uint8_t rtuDataStorage[255];
    Vector<uint8_t> rtuData(rtuDataStorage);
    uint8_t pduStorage[255];
    Vector<uint8_t> pdu(pduStorage);
}



#endif