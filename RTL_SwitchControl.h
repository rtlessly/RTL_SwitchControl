#pragma once
#ifndef _SwitchControl_h_
#define _SwitchControl_h_
/*******************************************************************************
Header file for the TaskManager class.
*******************************************************************************/
#include <RTL_Stdlib.h>
#include <RTL_Event.h>
#include <RTL_TaskManager.h>

/*******************************************************************************
  A component for reading the state of a switch or button connected to a GPIO
  pin on the Arduino. 
  
  The button/switch can is connected to a digital GPIO pin, and can use either the 
  the internal pull-up resistor or an external pull-down resistor.  By default, the
  internal pull-up resistor is used. If using an external pull-down resistor is used
  then pass false for the activeLow parameter to the constructor. 

  When activeLow is true, one terminal of the switch should be connected to the
  input pin and the other to ground. No external resistor is required. 

  Alternatively, if an activeLow is false, one terminal of the switch should be
  connected to the pin and the other to Vcc (+5v), and an external resistor should   
  be connected between the pin and ground. 

  Regardless of active low or active high, the SwitchControl class always returns
  1 when the switch is ON and 0 when the switch is OFF.
    
  Each SwitchControl object uses 9 bytes of data storage.
*******************************************************************************/
class SwitchControl : public TaskBase
{
    DECLARE_CLASSNAME;

public: 
    //**************************************************************************
    // Constants
    //**************************************************************************
    static constexpr EVENT_ID SWITCH_EVENT = (EventSourceID::Switch | EventCode::DefaultEvent);

    // The LSB indicates if the switch is ON (1) or OFF (0)
    // The 2nd LSB indicates if a state transition occurred to the state in the LSB
    enum : byte
    {
        OFF      = 0b00000000,   // steady state OFF (Alias for LOW)
        ON       = 0b00000001,   // steady state ON (Alias for HIGH)
        OPENED   = 0b00000010,   // transition to OFF state
        CLOSED   = 0b00000011,   // transition to ON state 
        RELEASED = 0b00000010,   // transition to OFF state (alias for OPENED)
        PRESSED  = 0b00000011,   // transition to ON state (alias for CLOSED)
        TOGGLED  = 0b00000010,   // Indicates a state transition occurred
    };

    //**************************************************************************
    // Events
    //**************************************************************************
public:
    Event StateChanged = Event(SWITCH_EVENT);

    //****************************************************************************
    // Constructor
    // By default, the constructor assumes the switch is using the internal pull-up
    // resistor. If the switch is using an external pull-down resistor then you
    // must explicitly pass false for the activeLow parameter.
    //****************************************************************************
public: 
    SwitchControl(uint8_t pin, bool activeLow=true);


    //****************************************************************************
    // Public Methods
    //****************************************************************************
public: 
    uint8_t Read();

    inline uint8_t ReadImmediate() { return readPin(); };

    inline bool IsActiveLow() { return _state.activeLow; }

    // TaskBase implementation
    void Poll() override;

    //****************************************************************************
    // Internal State
    //****************************************************************************
private: 
    struct // Occupies 1 byte
    {
        uint8_t pin              : 5; // Digital pin the switch is connected to
        uint8_t activeLow        : 1; // Indicates if the switch is active (ON) on LOW reading
        uint8_t currentState     : 1; // Current switch state (1=ON, 0=OFF)
    }
    _state;
    
    float _debounceFilter;

private:
    // Reads the switch input pin and inverts the sense if active low so that a 
    // return value of 1 always means the switch is activated (ON) and 0 means 
    // the switch is inactived (OFF).
    inline uint8_t readPin()
    {
        uint8_t pinState = digitalRead(_state.pin);

        return _state.activeLow ? (1 - pinState) : pinState;
    }
};

#endif
