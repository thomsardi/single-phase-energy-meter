#ifndef RELAY_CONTROL_H
#define RELAY_CONTROL_H

// #include <Arduino.h>

class RelayControl {
    public :
        RelayControl();
        int write(int line, bool state);
        ~RelayControl();
    private:
        int relaySet(int line);
        int relayReset(int line);

};

#endif