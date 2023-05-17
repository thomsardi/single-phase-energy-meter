#ifndef ENERGYMETERDATA_H
#define ENERGYMETERDATA_H

struct EnergyMeterFrequency
{
    int unit;
    int frequency;
    int voltageFrequency;
    int currentFrequency;
    int powerFrequency;
};

struct EnergyMeterCalibrator
{
    float coef_v;
    float coef_i;
    float coef_p;
};

struct EnergyMeterData
{
    int unit;
    int frequency;
    int voltage;
    int current;
    int power;
};

#endif