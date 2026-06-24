/*******************************************************************************
Implementation file for the SwitchControl class.
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
SwitchControl::SwitchControl(uint8_t _pin, bool activeLow)
{
    pin              = _pin;
    state.activeLow  = activeLow;
    state.longPress  = 0;
    state.repeating  = 0;

    // Initialize the switch pin.
    pinMode(_pin, activeLow ? INPUT_PULLUP : INPUT);

    // Get the initial pin state. lastPressTime is seeded with millis() so
    // that long-press timing is correct if the switch is already ON at startup.
    state.currentState = readPin();
    debounceFilter     = state.currentState;
    lastPressTime      = millis();
}


void SwitchControl::Poll()
{
    update();
}


uint8_t SwitchControl::update(uint32_t delayTime, const uint32_t repeatTime)
{
    //TRACE(Logger(_classname_, __func__) << endl;)
    auto now = millis();
    
    // Initialize the return value to the current switch state
    auto switchState = state.currentState | (state.longPress ? LONG_PRESS : 0);

    // read the switch input pin
    auto pinState = readPin();
        
    // Debouncing is performed by using a low-pass filter with latching. This is simpler
    // to implement and much more responsive than using a timeout. Mathematically, a 
    // low-pass filter is described by the following equation:
    //
    //      filtered = alpha*raw_value + (1 - alpha)*prev_filtered
    // 
    // Where filtered is the updated filtered value, prev_filtered is the filtered value
    // from the previous iteration, raw_value is the raw input data value, and alpha is 
    // the filter parameter. This equation is re-evaluated each time a new raw data value 
    // is read, with the current filtered value used as the "prev_filtered" value.
    // 
    // Alpha should have a value between 0.0 and 1.0. A strong low-pass filter will have
    // an alpha close to 0.0, typically around 0.1. This means that the prev_filtered
    // value dominates the raw_value causing the filtered value to change relatively 
    // slowly in response to the raw data. This is advantageous because if the raw_data
    // is noisy ("bouncy") we don't want to respond to all that noise, but only to the 
    // longer-term trend.
    static constexpr float alpha = 0.1F;
    static constexpr float beta  = 1.0F - alpha;
    
    debounceFilter = alpha*pinState + beta*debounceFilter;

#if DEBUG_PLOT==1
    // Output values for graphing on the plotting monitor
    Serial << pinState << " " << debounceFilter << endl;
#endif

    // The switch changes state only when the filter value exceed certain thresholds - this 
    // is the latching functionality. Two thresholds are defined, an upper threshold (0.9) and
    // a lower threshold (0.1). When the filtered value exceeds the upper threshold while the
    // current state is OFF then the switch toggles from off to on; and vice-versa when the 
    // filtered value drops below the lower threshold and the switch is currently ON. So once 
    // a state change occurs at one threshold, the state cannot change again until the filter
    // value exceeds the opposite threshold. Thus, transients across a threshold boundary 
    // cannot change the switch state once the it has toggled at that threshold.
    if (switchState == OFF && debounceFilter >= 0.9)
    {
        TRACE(Logger(_classname_, this) << F("TOGGLED ON: newState=") << pinState << F(", priorState=") << state.currentState << F(", switchState=") << switchState << endl;);
        switchState = PRESSED;
        state.currentState = ON;
        debounceFilter     = 1.0F;
        state.longPress    = 0;
        state.repeating    = 0;
        lastPressTime      = now;
        StateChanged.Post(this, switchState);
    }
    else if ((switchState & ON) && debounceFilter <= 0.1)
    {
        TRACE(Logger(_classname_, this) << F("TOGGLED OFF: newState=") << pinState << F(", priorState=") << state.currentState << F(", switchState=") << switchState << endl;);
        switchState = state.longPress ? LONG_PRESS_RELEASED : RELEASED;
        state.currentState = OFF;
        debounceFilter     = 0.0F;
        state.longPress    = 0;
        state.repeating    = 0;
        StateChanged.Post(this, switchState);
    }
    else if (state.currentState == ON && repeatTime > 0)
    {
        if (delayTime == 0) delayTime = 1000;
        
        if (!state.repeating && (now - lastPressTime) >= delayTime)
        {
            TRACE(Logger() << F("Repeat start press detected") << endl;);
            switchState      = PRESSED;
            state.repeating = 1;
            lastPressTime   = now;
            StateChanged.Post(this, switchState);
        }
        else if (state.repeating && (now - lastPressTime) >= repeatTime)
        {
            TRACE(Logger() << F("Repeated press detected") << endl;);
            switchState    = PRESSED;
            lastPressTime = now;
            StateChanged.Post(this, switchState);
        }
    }
    else if (state.currentState == ON && delayTime > 0 && !state.longPress)
    {
        if ((now - lastPressTime) >= delayTime)
        {
            TRACE(Logger() << F("Long press detected") << endl;);
            switchState         = LONG_PRESS_PRESSED;
            state.currentState = ON;
            state.longPress    = 1;
            StateChanged.Post(this, switchState);
        }
    }
    
    return switchState;
}


// Deprecated - long presses are now directly handled in update()
uint8_t SwitchControl::DetectLongPress(const uint32_t longPressTime)
{
    return DetectLongPress(update(), longPressTime);
}


// Deprecated - long presses are now directly handled in update()
uint8_t SwitchControl::DetectLongPress(uint8_t buttonState, const uint32_t longPressTime)
{
    auto newButtonState = buttonState;
    
    if ((buttonState & ON))
    {
        if ((millis() - lastPressTime) >= longPressTime && !state.longPress)
        {
            TRACE(Logger() << F("Long press detected") << endl;);
            state.longPress = 1;
            newButtonState = LONG_PRESS_PRESSED;
        }
    }
    else if (state.longPress)
    {
        TRACE(Logger() << F("Long press release detected") << endl;);
        state.longPress = 0;
        newButtonState = LONG_PRESS_RELEASED;
    }    

    return newButtonState;
}


//uint8_t SwitchControl::AltRead()
//{
//    //TRACE(Logger(_classname_, __func__) << endl;)
//
//    auto switchState = state.currentState;     // Return value is current switch state
//    auto pinState = digitalRead(state.pin); // read the switch input pin
//
//    if (pinState != state.debounceState)
//    {
//        state.debounceState = pinState;        // Set the new debounce state
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
//            TRACE(Logger(_classname_, this) << F("TOGGLED: newState=") << pinState << F(", currentState=") << state.currentState << endl;);
//
//            state.currentState = pinState;
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
    on the number line with wrap-around from the left to the right end. So,
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
    // The floating point version of the low-pass filter equation is:
    //
    //     filter_value = alpha*input + beta*prev_filter_value
    //
    // Where alpha is between 0.0 and 1.0 and beta = 1 - alpha.
    //
    // First, we use 255 to represent the "on" (1) state of the filter, and 0 to represent the 
    // off state. These values can be represented in a single byte, which saves considerable
    // memory (compared to floating-point values) yet is still big enough to give good
    // resolution to the filter. The filtered value will vary between these 2 extremes. 
    //
    // Second, we can define alpha and beta as integer values between 0 and 255 that represent 
    // the numerator of a fraction whose denominator is 255.
    //
    // Since beta = 1 - alpha, we can rewrite the filter equation as:
    // 
    //      beta*prev_filter_value = (1 - alpha)*prev_filter_value 
    //                             = prev_filter_value - alpha*prev_filter_value
    //
    // So:  filter_value = alpha*input + (prev_filter_value - alpha*prev_filter_value)
    //
    // Third, we can take advantage of "power-of-two" arithmetic to pick a value for the alpha 
    // parameter. We can use 1/16 (2**-4) as the alpha value, which is 0.0625, and corresponds 
    // to a beta value of 0.9375. Multiplying by 1/16 is the same as dividing by 16, which can 
    // be computed very rapidly by simply shifting a value right by 4 bits. Thus, we can replace
    // the the floating-point multiplication with very fast integer shift operation:
    // 
    //      beta*prev_filter_value = prev_filter_value - (prev_filter_value >> 4)
    // 
    // There is a slight complication - pure integer math introduces quantization effects that 
    // prevent the filter_value from ever rising above 240 (256-16) or below 15. To overcome
    // this, we must account for the remainder after dividing by 16. If the remainder is non-zero
    // then add 1 to the subtraction term (called the betaTerm). This can be done by converting
    // prev_filter_value to a 16-bit value and shifting the value into the high byte. When we divide 
    // it by 16 (shift right 4 bits) the remainder is shifted into the low byte. However, shifting 
    // prev_filter_value left 8 bits to move it into the high byte and then shifting it right again 
    // by 4 bits is the same as a net shift left of 4 bits. So a single shift left of 4 bits leaves
    // the correct quotient in the high byte and the remainder in the low byte. Whenever the low
    // byte is non-zero we just need to add 1 to the quotient.
    // 
    // Fourth, the raw data value is always either 0 (OFF) or 1 (ON), so the value of the first
    // term (alpha*input) can only ever be either 0 (0.0625 * 0) or 16 (0.0625 * 255). Thus, the 
    // floating-point arithmetic in the first term can be replaced with a simple integer 
    // multiplication of the pin state (1 or 0) and the constant value, 16. Further, since this
    // multiplication is a power-of-two, it can be replaced with a left-shift of 4 bits.
    
// auto betaTerm     = (uint16_t)prev_filter_value << 4;
// betaTerm          = highByte(betaTerm) + (lowByte(betaTerm) ? 1 : 0);
// auto filter_value = (prev_filter_value - betaTerm) + (pinState << 4);
