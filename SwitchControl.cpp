#define DEBUG 0

#include <Arduino.h>
#include <Debug.h>
#include "SwitchControl.h"


static DebugHelper Debug("SwitchControl");


//EVENT_ID SwitchControl::SWITCHCONTROL_EVENT = EventSource::GenerateEventID();


SwitchControl::SwitchControl(uint8_t pin, bool usePullUp, uint8_t debounceTime)
{
    _id = "SwitchControl";
    _state.pin = pin;
    _state.lastState = OFF;
    _state.debounceState = 0;
    _state.debounceTime = debounceTime;
    _state.inverted = usePullUp;

    // Initialize the switch pin. If usePullUp is true, then initialize with the
    // internal pull-up resistor. Otherwise, initialize it as a normal input
    // (which assumes an external pull-DOWN resistor is being used).
    //
    // When usePullUp is specified, one terminal of the switch should be connected
    // to the input pin and the other to ground. No external resistor is required.
    // HOWEVER, because the internal pull-up resistor is used, the "ON" state
    //(when the switch is closed) is LOW instead of HIGH. This class compensates
    // for that by inverting the signal before returning from the Read() method,
    // so that 1/ON means the switch is closed and 0/OFF means the switch is open.
    pinMode(_state.pin, usePullUp ? INPUT_PULLUP : INPUT);
}

uint8_t SwitchControl::Read()
{
    // default return value to last valid state read
    uint8_t switchState = _state.lastState;

    // read the switch input pin
    uint8_t state = digitalRead(_state.pin) == LOW ? OFF : ON;

    // Invert the switch state if the inverted flag is set
    if (_state.inverted) state = (state ^ ON);

    // Get the current time for the de-bounce timer
    unsigned long now = millis();

    // Whenever state changes we have to wait for the de-bounce interval to expire
    // before accepting it, so for now we just record the new state and reset the
    // de-bounce timer
    if (state != _state.debounceState)
    {
        _state.debounceState = state;
        _lastDebounceTime = now;

        Debug.Log(this) << __func__ << F(": Bounce! state=") << state << endl;
    }

    // If the last recorded de-bounce state is different from the last valid switch
    // switch state, AND the de-bounce timer has expired (which means the de-bounce
    // state has not changed for that duration) then we can accept the new state.
    // NOTE: If the state *did* change then the de-bounce timer was reset by the code
    //       above so this code won't execute (since now - _lastDebounceTime = 0),
    //       UNLESS the de-bounce interval is also 0 (which means no de-bouncing).
    if (_state.debounceState != _state.lastState)
    {
        if ((now - _lastDebounceTime) >= (unsigned long)_state.debounceTime)
        {
            _state.lastState = _state.debounceState;
            switchState = (_state.lastState == ON) ? CLOSED : OPENED;

            Debug.Log(this) << __func__ << F(": switchState=") << switchState << F(", lastState=") << _state.lastState << endl;

            QueueEvent(SWITCHCONTROL_EVENT, switchState);
        }
    }

    return switchState;
}


/*******************************************************************************
 Alternate Read routine that uses a different debounce algorithm.

 Returns the most recent stable state of the switch, 1 = closed, 0 = open.
*******************************************************************************/
//uint8_t SwitchControl::Read()
//{
//    return _state.lastState;
//}


void SwitchControl::Poll()
{
    Debug.Log(this) << __func__ << endl;

    Read();
//    // Alternate debounce algorithm
//    uint32_t now = millis();
//
//    if ((now - _lastDebounceTime) < 3) return;
//
//    _lastDebounceTime = now;
//
//    bool lastState = _state.lastState;
//
//    // read the switch input pin (and invert if necessary)
//    int reading = (digitalRead(_state.pin) == HIGH) ^ _state.inverted;
//
//    // Update debounce status
//    // XORing the current reading with the last stable state produces a 1
//    // when different and 0 when same. Then shift that result into the LSBit
//    // of _debounceState. As long as the switch is bouncing _debounceState
//    // will contain a mix of 1s and 0s. When the switch stabilizes in its
//    // new state _debounceState will eventually contains all 1s.
//    _debunceState = ((_debunceState << 1) | (reading ^ lastState));
//
//    // If the new switch state has been different than the last switch state
//    // for 16 consectutive readings then it has stabilized and we can flip
//    // the switch to the new state.
//    if (_debunceState == 0xff)
//    {
//         Debug.Log("Read => newState=%i, lastState=%i", (int)reading, (int)lastState);
//         _state.lastState = reading;
//         DispatchEvent(SWITCHCONTROL_EVENT, (int)lastState);
//    }
}

