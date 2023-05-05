#include "EnergyMeter.h"

/**
 * @brief   Class constructor
 * 
 * @param   pulseSignalPin    pulse signal input pin
 * @param   selPin            output pin to select mode between current or voltage measurement
 * @param   unit              PCNT module unit number (valid value are 0 - 7)
 * @param   channel           PCNT module channel number (valid value are 0 - 1)
*/
EnergyMeter::EnergyMeter(int pulseSignalPin, int selPin, int unit, int channel)
{
    _pulseSignalPin = pulseSignalPin;
    _selPin = selPin;
    _unit = unit;
    _channel = channel;
    pinMode(_selPin, OUTPUT);
}

/**
 * @brief   start the process
 *          @note              this method will init the PCNT module unit and then start the pulse counting
 * 
*/
void EnergyMeter::start()
{
    initPcnt();
    pcnt_counter_resume(_pcnt_unit[_unit]);
}

/**
 * @brief   initialize ESP32 PCNT module
 *          @note               this will configure the PCNT module, register the event and isr to handle an event
*/
void EnergyMeter::initPcnt()
{
    pcnt_config_t pcnt_config = {
    .pulse_gpio_num = _pulseSignalPin,    // set gpio for pulse input gpio
    .ctrl_gpio_num = -1,            // no gpio for control
    .lctrl_mode = PCNT_MODE_KEEP,   // when control signal is low, keep the primary counter mode
    .hctrl_mode = PCNT_MODE_KEEP,   // when control signal is high, keep the primary counter mode
    .pos_mode = PCNT_COUNT_INC,     // increment the counter on positive edge
    .neg_mode = PCNT_COUNT_DIS,     // do nothing on falling edge
    .counter_h_lim = _highLimit,     // set the higlimit counter
    .counter_l_lim = _lowLimit,             // set the lowlimit counter
    .unit = _pcnt_unit[_unit],               /*!< PCNT unit number */
    .channel = _pcnt_channel[_channel]
    }; 
    pcnt_unit_config(&pcnt_config); // pass the config parameter of pcnt
    // Enable the PCNT interrupt
    pcnt_event_enable(_pcnt_unit[_unit], PCNT_EVT_H_LIM); // register the event callback when counter reached the maximum counter limit according to pcnt_config

    pcnt_counter_pause(_pcnt_unit[_unit]); // pause the counter of pcnt unit
    pcnt_counter_clear(_pcnt_unit[_unit]); // clear the counter of pcnt unit

    pcnt_isr_register(_pcnt_handler, NULL, 0, NULL); // register isr handler, when the event is triggered, the isr will be executed
    pcnt_intr_enable(_pcnt_unit[_unit]); // enable the interrupt of pcnt
    // Start the PCNT counter
    // pcnt_counter_resume(_pcnt_unit[_unit]); // resume the pcnt counting pulse
}

/**
 * @brief   register the handler when pcnt event triggered
 * @param   pointer             to function which takes a pointer argument and return nothing
 *          @note               *pcnt_handler is the name of function, void * is a pointer argument
*/
void EnergyMeter::registerHandler(void (*pcnt_handler)(void *))
{
    _pcnt_handler = pcnt_handler;
}

/**
 * @brief   update the value in the class
 * @param   intr_status     origin of the pcnt module unit
 *          @note           when using multiple pcnt module, each event on the pcnt module will be handled by the same event handler
 *                          in order to verify if the triggered event is from the same pcnt in the class config. it needs to be
 *                          checked first. if it is same as config, it then updates the value
 *                          e.g :
 *                          PCNT_UNIT_0 intr_status value is 2^0 = 1
 *                          PCNT_UNIT_1 intr_status value is 2^1 = 2
 *                          ...
 *                          ...
 *                          PCNT_UNIT_7 intr_status value is 2^7 = 256
 *                          by evaluate the intr_status, 
*/
void EnergyMeter::update(uint32_t intr_status)
{
    if (intr_status & (BIT(_unit)))
    {
        _energyMeterData.unit = _unit;
        _mult++;
    }
}

void EnergyMeter::setMode(uint8_t mode)
{
    _mode = mode;
    if (_mode < 2)
    {
        digitalWrite(_selPin, mode);
    }
}

void EnergyMeter::pause()
{
    pcnt_counter_pause(_pcnt_unit[_unit]);
}

void EnergyMeter::clear()
{
    pcnt_counter_clear(_pcnt_unit[_unit]);
}

void EnergyMeter::resume()
{
    pcnt_counter_resume(_pcnt_unit[_unit]);
}

void EnergyMeter::calculate()
{
    int16_t pulse = 0;
    int current;
    int voltage;
    int power;
    pcnt_get_counter_value(_pcnt_unit[_unit], &pulse);
    int frequency = (_mult*_highLimit) + pulse;
    _energyMeterData.frequency = frequency;
    switch (_mode)
    {
        case 0:
            current = static_cast<int>(frequency*1000*_coef_i);
            _energyMeterData.current = current;
            break;
        case 1:
            voltage = static_cast<int>(frequency*1000*_coef_v);
            _energyMeterData.voltage = voltage;
            break;
        
        default:
            power = static_cast<int>(frequency*1000*_coef_p);
            _energyMeterData.power = power;
    }
    _mult = 0;
}

EnergyMeterData EnergyMeter::getEnergyMeterData()
{
    return _energyMeterData;
}