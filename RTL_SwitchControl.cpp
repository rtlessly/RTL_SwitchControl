/*******************************************************************************
Implementation file for the TaskManager class.
*******************************************************************************/
#define DEBUG 0
#define DEBUG_PLOT 0

#include <Arduino.h>
#include <RTL_Stdlib.h>
#include "RTL_SwitchControl.h"


DEFINE_CLASSNAME(SwitchControl);


//******************************************************************************
// Constructor
// If activeLow is true (default), then the digital pin is initialized with the
// internal pull-up resistor. Otherwise, it is initialized as a normal input,
// which assumes an external pull-down resistor is being used.
//******************************************************************************
SwitchControl::SwitchControl(uint8_t pin, bool activeLow)
{
    _state.pin       = pin;
    _state.activeLow = activeLow;
    _state.longPress = 0;

    // Initialize the switch pin.
    pinMode(pin, activeLow ? INPUT_PULLUP : INPUT);

    // Get the initial pin state
    _state.currentState = readPin();
    _debounceFilter     = _state.currentState;
    _lastPressTime      = 0xFFFFFFFF;
}


void SwitchControl::Poll()
{
    Read();
}


uint8_t SwitchControl::Read()
{
    //TRACE(Logger(_classname_, __func__) << endl;)

    // Initialize the return value to the current switch state
    auto switchState = _state.currentState;

    // read the switch input pin
    auto pinState = readPin();
        
    // Debouncing is performed by using a low-pass filter. This is simpler to implement 
    // and much more responsive than using a timeout. Mathematically, a low-pass filter
    // is described by the following equation:
    //
    //      filtered = alpha*filtered_old + (1 - alpha)*raw_value
    // 
    // Where filtered is the updated filtered value, filtered_old is the filtered value
    // from the previous iteration, raw_value is the raw data value read from the input,
    // and alpha is the filter parameter. This equation is re-evaluated each time a new
    // raw data value is read, with the current filtered value used as the "filtered_old"
    // value.
    // 
    // Alpha should have a value between 0.0 and 1.0. A strong low-pass filter will have
    // an alpha close to 1.0, typically at or above 0.9 (In this implementation an alpha
    // of 0.95 is used). This means that the filtered_old value dominates the raw_value 
    // causing the filtered value to change relatively slowly in response to the raw data.
    // This is advantageous because if the raw data is noisy ("bouncy") we don't want to
    // respond to all that noise, but only to the longer-term trend.
    _debounceFilter = 0.95*_debounceFilter + 0.05*pinState;

#if DEBUG_PLOT==1
    // Output values for graphing on the plotting monitor
    Serial << pinState << " " << _debounceFilter << endl;
#endif

    // The switch changes state only when the filter value exceed certain thresholds. Two thresholds 
    // are defined, an upper threshold (0.9) and a lower threshold (0.1). When the filtered value
    // exceeds the upper threshold while the current state is OFF then the switch toggles from off
    // to on; and vice-versa when the filtered value drops below the lower threshold and the switch 
    // is currently ON. So once a state change occurs at one threshold, the state cannot change 
    // again until the filter value exceeds the opposite threshold. Thus, transients across a 
    // threshold cannot change the switch state once the switch has toggled at that threshold.
    if (switchState == OFF && _debounceFilter > 0.9)
    {
        switchState = PRESSED;
        TRACE(Logger(_classname_, this) << F("TOGGLED ON: newState=") << pinState << F(", priorState=") << _state.currentState << F(", switchState=") << switchState << endl;);
        _state.currentState = ON;
        _state.longPress = 0;
        _lastPressTime = millis();
        _debounceFilter = _state.currentState;
        StateChanged.Post(this, switchState);
    }
    else if (switchState == ON && _debounceFilter < 0.1)
    {
        switchState = RELEASED;
        TRACE(Logger(_classname_, this) << F("TOGGLED OFF: newState=") << pinState << F(", priorState=") << _state.currentState << F(", switchState=") << switchState << endl;);
        _state.currentState = OFF;
        _debounceFilter = _state.currentState;
        StateChanged.Post(this, switchState);
    }

    return switchState;
}


uint8_t SwitchControl::DetectLongPress(const uint32_t longPressTime)
{
    return DetectLongPress(Read(), longPressTime);
}


uint8_t SwitchControl::DetectLongPress(uint8_t buttonState, const uint32_t longPressTime)
{
    auto newButtonState = buttonState;
    
    if ((buttonState & ON))
    {
        if ((millis() - _lastPressTime) >= longPressTime && !_state.longPress)
        {
            TRACE(Logger() << F("Long press detected") << endl;);
            _state.longPress = 1;
            newButtonState = LONG_PRESS_PRESSED;
        }
    }
    else if (_state.longPress)
    {
        TRACE(Logger() << F("Long press release detected") << endl;);
        _state.longPress = 0;
        newButtonState = LONG_PRESS_RELEASED;
    }    

    return newButtonState;
}


//uint8_t SwitchControl::AltRead()
//{
//    //TRACE(Logger(_classname_, __func__) << endl;)
//
//    auto switchState = _state.currentState;     // Return value is current switch state
//    auto pinState = digitalRead(_state.pin); // read the switch input pin
//
//    if (pinState != _state.debounceState)
//    {
//        _state.debounceState = pinState;        // Set the new debounce state
//        _debounceTimer = (lowByte(millis()) + _debounceInterval) | 0x01;     // Set the debounce timeout (but make sure its not zero)
//        TRACE(Logger(_classname_, this) << F("Bounce! state=") << pinState << F(", _debounceTimer=") << _debounceTimer << endl;);
//    }
//    else if (_debounceTimer > 0)    // If we are timing a debounce
//    {
//        auto clock = lowByte(millis());
//
//        TRACE(Logger(_classname_, this) << F("clock=") << clock << F(", Timer=") << (uint8_t)(_debounceTimer - clock) << endl;);
//
//        // If the debounce time has expired then the new switch state has stablized so accept it.
//        // See note 1 at the end of this file regarding the usigned subtraction done here.
//        if ((uint8_t)(_debounceTimer - clock) > _debounceInterval)
//        {
//            _debounceTimer = 0;
//
//            TRACE(Logger(_classname_, this) << F("TOGGLED: newState=") << pinState << F(", currentState=") << _state.currentState << endl;);
//
//            _state.currentState = pinState;
//            switchState = pinState | TOGGLED;
//            StateChanged.Post(this, switchState);
//        }
//    }
//
//    return switchState;
//}


/*******************************************************************************
 NOTES:
 1. The difference between the clock and the timeout is treated as an 8-bit 
    unsigned value. This converts negative differences into positive values with
    wrap-around, so the difference is the leftward distance between the two
    numbers, where "leftward" means to always measure distance toward the left
    on the number line with wrap-wround from the left to the right end. So,
    0x02 - 0xFE = 4, since the leftward distance between the numbers (via wrap-
    around) is 4. likewise, 0xFE - 0x02 = 0xFB since that is the leftward
    distance from 0xFE to 0x02 is 0xFB.

      0x02                              0xFE - 0x02 = 0xFB                             0xFE
      |<-------------------------------------------------------------------------------|
 0x0|-|---------------------------------- number line ---------------------------------|-|0xFF
  +<--|                                                                                |<--+
  |                                     0x02 - 0xFE = 0x04                                 |
  +--->--------------------->--------------wrap-around---->--------------->--------------->+

*******************************************************************************/


    // In this implementation we take advantage of several optimizations since we know we 
    // are dealing with binary data, i.e., the switch can only have a value of on (1) or
    // off (0) and nothing in-between. This allows us to perform the math using fast integer
    // arithmetic instead of slower floating-point arithmetic. 
    // 
    // First, we use 255 to represent the "on" (1) state of the filter, and 0 to represent the 
    // off state. These values can be represented in a single byte, which saves considerable
    // memory (compared to floating-point values) yet is still big enough to give good
    // resolution to the filter. The filtered value will vary between these 2 extremes. 
    //
    // Second, we can define an alternate parameter, called beta, that is 1 - alpha.
    // 
    //      beta = 1 - alpha
    // or
    //      alpha = 1 - beta
    // 
    // The first term of the filter equation can then be rewritten in terms of beta:
    // 
    //      alpha*filtered_old = (1 - beta)*filtered_old 
    //                         = filtered_old - beta*filtered_old
    //
    // Third, we can take advantage of "power-of-two" arithmetic to pick a value for the beta 
    // parameter. We can use 1/32 (2**-5) as the beta value, which is very close to 0.03. That 
    // corresponds to an alpha value of 0.97. Multiplying by 1/32 is the same as dividing by 32, 
    // which can be computed very rapidly by simply shifing a value right by 5 bits. Thus, we 
    // can replace the the floating-point multiplicaton in the first term with very fast integer 
    // shift and subtract operations:
    // 
    //      alpha*filtered_old = filtered_old - (filtered_old >> 5)
    // 
    // There is a slight complication - pure integer math introduces quantization effects that 
    // prevent the filtered valued from ever rising above 224 (256-32) or below 31. To overcome
    // this, we must account for the remainder after dividing by 32. If the remainder is non-zero
    // then add 1 to the subtraction term (called the alphaTerm). This can be done by converting
    // filtered_old to a 16-bit value and shifting the value into the high byte. When we divide 
    // it by 32 (shift right 5 bits) the remainder is shifted into the low byte. However, shifting 
    // filtered_old left 8 bits to move it into the high byte and then shifting it right again by
    // 5 bits is the same as a net shift left of 3 bits. So a single sift left of 3 bits leave the
    // correct quotient in the high byte and the remainder in the low byte. Whenever the low byte 
    // is non-zero we just need to add 1 to the quotient.
    // 
    // Fourth, the raw data value is always either 0 (OFF) or 1 (ON), so the value of the second
    // term can only ever be either 0 (0.03 * 0) or 8 (0.03 * 255). Thus, the floating-point 
    // arithmetic in the second term can be replaced with a simple interger multiplication of the 
    // pin state (1 or 0) and the constant value, 8. 
//auto alphaTerm = (uint16_t)_debounceFilter << 3;
//
//alphaTerm = highByte(alphaTerm) + (lowByte(alphaTerm) ? 1 : 0);
//_debounceFilter = (_debounceFilter - alphaTerm) + (pinState * 8);
