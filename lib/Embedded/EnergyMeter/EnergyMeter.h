#ifndef ENERGYMETER_H
#define ENERGYMETER_H

// #define ENERGYMETER_DEBUG

// #ifdef ENERGYMETER_DEBUG
//     #define LOG_PRINT(x)    Serial.print(x)
//     #define LOG_PRINTLN(x)  Serial.println(x)
// #else
//     #define LOG_PRINT(x)
//     #define LOG_PRINTLN(x)
// #endif

#include <Arduino.h>
#include "EnergyMeterData.h"
#include "driver/pcnt.h"
#include "soc/pcnt_struct.h"

class EnergyMeter {
    public :
        EnergyMeter(int pulseSignalPin, int selPin, int unit, int channel);
        void start();
        void update(uint32_t intr_status);
        void registerHandler(void (*pcnt_handler)(void *));
        void setCalibrator (float coef_v, float coef_i, float coef_p);
        void setCalibrator (const EnergyMeterCalibrator &energyMeterCalibrator);
        void setMode(uint8_t mode);
        void calculate();
        void pause();
        void clear();
        void resume();
        int getMode();
        EnergyMeterData getEnergyMeterData();

    private :
        void initPcnt();

        void (*_pcnt_handler)(void *) = NULL;
        bool _isNoSelPin;
        uint8_t _mode;
        int16_t _lowLimit = 0;
        int16_t _highLimit = 30000;
        int _pulseSignalPin;
        int _selPin;
        int _unit;
        int _channel;
        int _mult;
        float _coef_v = 1;
        float _coef_i = 1;
        float _coef_p = 1;
        pcnt_unit_t _pcnt_unit[8] = {
            PCNT_UNIT_0,
            PCNT_UNIT_1,
            PCNT_UNIT_2,
            PCNT_UNIT_3,
            PCNT_UNIT_4,
            PCNT_UNIT_5,
            PCNT_UNIT_6,
            PCNT_UNIT_7
        };
        pcnt_channel_t _pcnt_channel[2] = {
            PCNT_CHANNEL_0,
            PCNT_CHANNEL_1
        };
        EnergyMeterFrequency _energyMeterFrequency;
};


#endif