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
    if (_selPin < 0)
    {
        _isNoSelPin = true;
    }
    else
    {
        pinMode(_selPin, OUTPUT);
    }
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
*/
void EnergyMeter::update(uint32_t intr_status)
{
    if (intr_status & (BIT(_unit)))
    {
        _energyMeterFrequency.unit = _unit;
        _mult++;
    }
}

/**
 * @brief   set calibrator value
 * @param   coef_v  voltage coefficient
 *          @note   to get this value, use this formula coef_v = voltage reading on voltmeter / reading frequency
 * @param   coef_i  current coefficient
 *          @note   to get this value, use this formula coef_i = current reading on amperemeter / reading frequency
 * @param   coef_p  power coefficient
 *          @note   to get this value, use this formula coef_p = power reading on wattmeter / reading frequency
*/
void EnergyMeter::setCalibrator(float coef_v, float coef_i, float coef_p)
{
    _coef_v = coef_v;
    _coef_i = coef_i;
    _coef_p = coef_p;
}

/**
 * brief    set calibrator value
 * @param   energyMeterCalibrator
 *          @note   pass the struct of EnergyMeterCalibrator configuration to configure the calibrator
*/
void EnergyMeter::setCalibrator(const EnergyMeterCalibrator &energyMeterCalibrator)
{
    _coef_v = energyMeterCalibrator.coef_v;
    _coef_i = energyMeterCalibrator.coef_i;
    _coef_p = energyMeterCalibrator.coef_p;
}

/**
 * @brief   set the mode for current or voltage rms measurement
 * @param   mode    set the mode, 0 for current measurement, 1 for voltage measuremen, other value for power measurement
*/
void EnergyMeter::setMode(uint8_t mode)
{
    _mode = mode;
    if (_isNoSelPin)
    {
        _mode = 2;
        return;
    }
    if (_mode < 2 && _mode >= 0)
    {
        if(_mode == 0)
        {
            // LOG_PRINTLN("Current Mode");
        }
        else
        {
            // LOG_PRINTLN("Voltage Mode");
        }
        if (!_isNoSelPin)
        {
            digitalWrite(_selPin, mode);
        }
    }
    else
    {
        _mode = 2;
        // LOG_PRINTLN("Power Mode");
    }
}

/**
 * @brief   pause the pcnt counting pulse
*/
void EnergyMeter::pause()
{
    pcnt_counter_pause(_pcnt_unit[_unit]);
}

/**
 * @brief   clear the counter of pcnt
*/
void EnergyMeter::clear()
{
    pcnt_counter_clear(_pcnt_unit[_unit]);
}

/**
 * @brief   resume the pcnt counting pulse
*/
void EnergyMeter::resume()
{
    pcnt_counter_resume(_pcnt_unit[_unit]);
}

/**
 * @brief   calculate the frequency and convert it into voltage, current, power depending on the mode selected
*/
void EnergyMeter::calculate()
{
    int16_t pulse = 0;
    int current;
    int voltage;
    int power;
    pcnt_get_counter_value(_pcnt_unit[_unit], &pulse);
    int frequency = (_mult*_highLimit) + pulse;
    _energyMeterFrequency.frequency = frequency;
    switch (_mode)
    {
        case 0:
            // current = static_cast<int>(frequency*1000*_coef_i);
            // current = (frequency*1000*_coef_i);
            _energyMeterFrequency.currentFrequency = frequency;
            break;
        case 1:
            // voltage = static_cast<int>(frequency*1000*_coef_v);
            // voltage = (frequency*1000*_coef_v);
            _energyMeterFrequency.voltageFrequency = frequency;
            break;
        
        default:
            // power = static_cast<int>(frequency*1000*_coef_p);
            // power = (frequency*1000*_coef_p);
            _energyMeterFrequency.powerFrequency = frequency;
    }
    _mult = 0;
}

/**
 * @brief   get mode
 * @return  mode of the class
*/
int EnergyMeter::getMode()
{
    return _mode;
}

/**
 * @brief   get the energy meter data 
 * @return  EnergyMeterData struct which consist of pcnt unit number, frequency, voltage, current, power
*/
EnergyMeterData EnergyMeter::getEnergyMeterData()
{
    EnergyMeterData energyMeterData;
    energyMeterData.unit = _energyMeterFrequency.unit;
    energyMeterData.frequency = _energyMeterFrequency.frequency;
    energyMeterData.current = _energyMeterFrequency.currentFrequency * 1000 * _coef_i;
    energyMeterData.voltage = _energyMeterFrequency.voltageFrequency * 1000 * _coef_v;
    energyMeterData.power = _energyMeterFrequency.powerFrequency * 1000 * _coef_p;
    return energyMeterData;
}