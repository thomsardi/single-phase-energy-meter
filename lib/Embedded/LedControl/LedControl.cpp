#include "LedControl.h"

LedControl::LedControl()
{

}

void LedControl::write(int line, bool state, CRGB leds[])
{
    if (line <= 0)
    {
        return;
    }

    if(state > 0)
    {
        leds[line-1] = CRGB::Green;
        return;
    }
    leds[line-1] = CRGB::Red;
    return;
}

LedControl::~LedControl()
{

}