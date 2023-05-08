#ifndef ENERGYMETERDATA_H
#define ENERGYMETERDATA_H

struct EnergyMeterData
{
    int unit;
    int frequency;
    int voltage;
    int current;
    int power;
};

struct EnergyMeterCalibrator
{
    float coef_v;
    float coef_i;
    float coef_p;
};

#endif