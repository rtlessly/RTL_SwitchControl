#ifndef _SwitchControl_h_
#define _SwitchControl_h_

#include <EventSource.h>


/*******************************************************************************
  A component for reading the state of a switch or button connected to a GPIO
  pin on the Arduino. 
  
  This class assumes that the button/switch is connected to a digital GPIO pin 
  using the internal pull-up resistor. In this configuration, one terminal of the
  button/switch is connected to the GPIO pin and the other to ground. No external 
  resistor is required as the internal pull-up resistor in used.
  
  Note that because the internal pull-up resistor is used (which pulls the signal 
  level to 5V when the button is open), the GPIO pin will read HIGH when the button 
  is open (OFF) and LOW when the button is closed (ON). This class compensates for
  that by inverting the signal before returning from the Read() method, so return
  values conform to conventional expectations (i.e., 1/ON=closed/pressed, 0/OFF=open/not pressed).
  
  Each SwitchControl object uses 8 bytes of data storage.
*******************************************************************************/
class SwitchControl : public EventSource
{
    public: static EVENT_ID SWITCHCONTROL_EVENT;
    
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
    // The constructor assigns a default de-bounce interval of 2 ms. If you want
    // a different value then you can pass your desired value (in milliseconds) as 
    // the third parameter. In addition, if you do not want de-bouncing then you 
    // MUST pass 0 as the third parameter.
    //****************************************************************************
    public: SwitchControl(uint8_t pin, bool usePullUp=true, uint8_t debounceTime = 2);

    //****************************************************************************
    // Public Methods
    //****************************************************************************
    public: uint8_t Read();

    //****************************************************************************
    // Internal State
    //****************************************************************************
    private: struct // Occupies 2 bytes
    {
    uint8_t pin           : 4; // Digital pin switch is connected to
    uint8_t lastState     : 1; // last valid switch state (1=ON, 0=OFF)
    uint8_t debounceState : 1; // Last switch state read for de-bounce (1=ON, 0=OFF)
    uint8_t inverted      : 1; // indicates if the switch signal is inverted (i.e., LOW=ON, HIGH=OFF)
    uint8_t unused        : 1; // Padding to force alignment to byte boundary
    uint8_t debounceTime  : 8; // De-bounce time period (0-255ms)
    }
    _state;

    // Time of last state change for de-bounce timer
    private: unsigned long _lastDebounceTime;

    //****************************************************************************
    // EventSource implementation
    //****************************************************************************
    public: virtual void Poll();
};

#endif
