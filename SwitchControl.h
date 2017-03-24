#ifndef _SwitchControl_h_
#define _SwitchControl_h_

#include <EventSource.h>


/*******************************************************************************
  A component for reading the state of a switch or button connected to a GPIO
  pin on the Arduino. 
  
  The button/switch can is connected to a digital GPIO pin, and can use either the 
  the internal pull-up resistor or an external pull-down resistor.  By default, the
  internal pull-up resistor is used. If using an external pull-down resistor then 
  pass false as the second argument to the constructor. In either case, the Read() 
  method is designed to return true when the switch is in closed state.
  
  Note that when the internal pull-up resistor is used (which pulls the signal 
  level to 5V when the button is open), the GPIO pin will read HIGH when the button 
  is open (OFF) and LOW when the button is closed (ON). The Read()method compensates
  for that by inverting the signal before returning from the Read() method, so 
  return values conform to conventional expectations (i.e., 1/ON=closed/pressed, 
  0/OFF=open/not pressed).
  
  Each SwitchControl object uses 8 bytes of data storage.
*******************************************************************************/
class SwitchControl : public EventSource
{
    public: const EVENT_ID SWITCHCONTROL_EVENT = (EventSourceID::Switch | EventCode::DefaultEvent);
    
    // The LSB indicates if the switch is ON (1) or OFF (0)
    // The 2nd LSB indicates if a state transition occurred to the state in the LSB
    public: enum SwitchControlState
    {
        OFF      = 0b00000000,   // steady state OFF
        ON       = 0b00000001,   // steady state ON
        OPENED   = 0b00000010,   // transition to OFF state
        CLOSED   = 0b00000011,   // transition to ON state 
        RELEASED = 0b00000010,   // transition to OFF state (alias for OPENED)
        PRESSED  = 0b00000011,   // transition to ON state (alias for CLOSED)
    };

    //****************************************************************************
    // Constructor
    // By default, the constructor assumes the switch is using the internal pull-up
    // resistor. If the switch is using an external pull-down resistor then you
    // must explicitly pass false as the second parameter.
    //
    // The constructor assigns a default de-bounce interval of 50 ms. If you want
    // a different value then you can pass your desired value (in milliseconds) as 
    // the third parameter. In addition, if you do not want de-bouncing then you 
    // MUST pass 0 as the third parameter.
    //****************************************************************************
    public: SwitchControl(uint8_t pin, bool usePullUp=true, uint8_t debounceTime = 50);

    //****************************************************************************
    // Public Methods
    //****************************************************************************
    public: uint8_t Read();

    //****************************************************************************
    // Internal State
    //****************************************************************************
    private: struct // Occupies 2 bytes
    {
    uint8_t pin           : 5; // Digital pin switch is connected to
    uint8_t lastState     : 1; // last valid switch state (1=ON, 0=OFF)
    uint8_t debounceState : 1; // Last switch state read for de-bounce (1=ON, 0=OFF)
    uint8_t inverted      : 1; // indicates if the switch signal is inverted (i.e., LOW=ON, HIGH=OFF)
    uint8_t debounceTime  : 8; // De-bounce time period (0-255ms)
    }
    _state;

    private: uint8_t _debounceState;
    
    // Time of last state change for de-bounce timer
    private: unsigned long _lastDebounceTime;

    //****************************************************************************
    // EventSource implementation
    //****************************************************************************
    public: virtual void Poll();
};

#endif
