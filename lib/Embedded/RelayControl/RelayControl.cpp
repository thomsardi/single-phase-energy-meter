#include "RelayControl.h"

RelayControl::RelayControl()
{

}

int RelayControl::write(int line, bool state)
{
    if (state > 0)
    {
        return relaySet(line);
    }
    return relayReset(line);
}

int RelayControl::relaySet(int line)
{
    int lineToPin = line - 1;
    int multiplier = lineToPin / 3;
    int totalPin = (8 + 8*(multiplier)) - 1;
    int substract = (lineToPin - (multiplier*3)) * 2;
    int pin = totalPin - substract;
    return pin;
}

int RelayControl::relayReset(int line)
{
    int pin = relaySet(line);
    return pin - 1;
}

RelayControl::~RelayControl()
{

}