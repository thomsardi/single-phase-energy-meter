#ifndef LED_CONTROL_H
#define LED_CONTROL_H

#include <Arduino.h>
#include <FastLED.h>

class LedControl {
    public :
        LedControl();
        void write(int line, bool state, CRGB leds[]);
        ~LedControl();
    private:

};

#endif