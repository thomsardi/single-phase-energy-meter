#ifndef MODBUSRTU_H
#define MODBUSRTU_H

// #define SERIALHANDLER_DEBUG

// #ifdef SERIALHANDLER_DEBUG
//     #define LOG_PRINT(x)    Serial.print(x)
//     #define LOG_PRINTLN(x)  Serial.println(x)
// #else
//     #define LOG_PRINT(x)
//     #define LOG_PRINTLN(x)
// #endif

#include <Arduino.h>

class ModbusRTU {
    public :
        // SerialHandler(Stream *serial);
        ModbusRTU();
        setSlaveId();
        void write();
    private :

};


#endif