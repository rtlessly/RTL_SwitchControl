#pragma once
#ifndef _SwitchControl_h_
#define _SwitchControl_h_
/*******************************************************************************
Header file for the SwitchControl class.
*******************************************************************************/
#include <RTL_Stdlib.h>
#include <RTL_Event.h>
#include <RTL_TaskManager.h>

/*******************************************************************************
  A component for reading the state of a switch or pushbutton connected to a GPIO
  pin on the Arduino, with appropriate debouncing. 
  
  The button/switch can is connected to a digital GPIO pin, and can use either the 
  the internal pull-up resistor or an external pull-down resistor.  By default, the
  internal pull-up resistor is used. If using an external pull-down resistor then
  pass `false` for the `activeLow` parameter to the constructor. 

  When `activeLow` is `true`, one terminal of the switch should be connected to the
  input pin and the other to ground. No external resistor is required. 

  Alternatively, if an `activeLow` is `false`, one terminal of the switch should be
  connected to the input pin and the other to Vcc (+5v), and an external resistor 
  (10K - 50K) should be connected between the pin and ground. 

  Regardless of active low or active high, the `SwitchControl` class always returns
  1 when the switch is ON and 0 when the switch is OFF.
    
  Each `SwitchControl` object uses 10 bytes of data storage on an ATMega328P (i.e.,
  an Arduino Uno or similar 8-bit processors). More space may be used on other
  micro-controller architectures that require word-boundary alignments.
*******************************************************************************/
class SwitchControl : public TaskBase
{
    DECLARE_CLASSNAME;

public: 
    //**************************************************************************
    // Constants
    //**************************************************************************
    static constexpr EVENT_ID SWITCH_EVENT = (EventSourceID::Switch | EventCode::DefaultEvent);

    // Switch states. 
    // - The LSB indicates if the switch is ON (1) or OFF (0)
    // - The 2nd LSB indicates if a state transition occurred to the state in the LSB.
    // - The MSB indicates if a long press was detected.
    enum : uint8_t
    {
        OFF        = 0b00000000,   // steady state OFF (Alias for LOW)
        ON         = 0b00000001,   // steady state ON (Alias for HIGH)
        TOGGLED    = 0b00000010,   // Indicates a state transition occurred
        OPENED     = 0b00000010,   // Transition to OFF state
        CLOSED     = 0b00000011,   // Transition to ON state 
        TOGGLE_OFF = OPENED,       // Transition to OFF state (Alias for OPENED)
        TOGGLE_ON  = CLOSED,       // Transition to ON state (Alias for CLOSED)
        RELEASED   = OPENED,       // Transition to OFF state (alias for OPENED)
        PRESSED    = CLOSED,       // Transition to ON state (alias for CLOSED)
        LONG_PRESS = 0b10000000,   // Indicates a long press was detected.
        LONG_PRESS_PRESSED  = LONG_PRESS | PRESSED,
        LONG_PRESS_ON       = LONG_PRESS | ON,
        LONG_PRESS_RELEASED = LONG_PRESS | RELEASED,
    };

    
    //**************************************************************************
    // Events
    //**************************************************************************
public:
    Event StateChanged = Event(SWITCH_EVENT);

    /****************************************************************************
     * Creates a `SwitchControl` instance bound to the specified GPIO pin.
     *
     * Configures the pin as `INPUT_PULLUP` when `activeLow` is `true`, or as
     * plain `INPUT` when `activeLow` is `false` (external pull-down assumed).
     *
     * @param _pin      The GPIO pin number the switch is wired to.
     * @param activeLow `true` (default) for active-low wiring using the
     *                  internal pull-up resistor; `false` for active-high
     *                  wiring with an external pull-down resistor.
     ****************************************************************************/
public:
    SwitchControl(uint8_t _pin, bool activeLow=true);


    /****************************************************************************
     * Reads the current switch state with debouncing and optional repeat/long-
     * press detection.
     *
     * When called from `Poll()` (both arguments at their defaults), performs
     * a single debounced read and fires `StateChanged` on any transition.
     *
     * **Long-press mode** (`delayTime` > 0, `repeatTime` == 0): fires a
     * `LONG_PRESS_PRESSED` event after the switch has been held for
     * `delayTime` ms.
     *
     * **Auto-repeat mode** (`repeatTime` > 0): fires the first `PRESSED` event
     * after an initial delay, then another every `repeatTime` ms while the
     * switch remains held. The initial delay is `delayTime` ms if non-zero,
     * or 1000 ms if `delayTime` is `0`. Auto-repeat takes priority over
     * long-press mode — a long press would always be detected before the first
     * repeat could occur since both use the same delay threshold.
     *
     * @param delayTime  Milliseconds before a long-press or first auto-repeat
     *                   event is generated. In auto-repeat mode, defaults to
     *                   1000 ms when `0`. In long-press mode, `0` disables
     *                   the feature entirely.
     * @param repeatTime Milliseconds between subsequent auto-repeat `PRESSED`
     *                   events. `0` selects long-press mode.
     * @return A bitmask composed of the switch-state constants (`OFF`, `ON`,
     *         `PRESSED`, `RELEASED`, `LONG_PRESS_PRESSED`, etc.).
     ****************************************************************************/
    uint8_t update(uint32_t delayTime=0, const uint32_t repeatTime=0);

    /****************************************************************************
     * Reads the raw pin state immediately, bypassing the debounce filter.
     *
     * Useful for sampling the pin state at a precise moment without waiting
     * for the filter to settle. The active-low sense is still applied, so the
     * return value follows the same `ON`/`OFF` convention as `update()`.
     *
     * @return `ON` (1) if the switch is currently activated; `OFF` (0) if not.
     ****************************************************************************/
    inline uint8_t ReadImmediate() { return readPin(); };

    /****************************************************************************
     * Returns whether the `SwitchControl` is expecting the switch to be wired in
     * the active-low configuration (the pin the switch is connected to indicates
     * LOW when the switch is pressed).
     *
     * @return `true` if the switch is active-low (internal pull-up in use);
     *         `false` if the switch is active-high (external pull-down in use).
     ****************************************************************************/
    inline bool IsActiveLow() { return state.activeLow; }

    /****************************************************************************
     * Reads the switch and detects a long press in a single call.
     *
     * **Deprecated** — long-press detection is now handled directly inside
     * `update()` via its `delayTime` parameter. Prefer using `update(delayTime)`.
     *
     * @param longPressTime Milliseconds the switch must be held before a
     *                      long-press is reported. Defaults to 1000 ms.
     * @return A bitmask composed of the switch-state constants. Returns
     *         `LONG_PRESS_PRESSED` when a long press is first detected, and
     *         `LONG_PRESS_RELEASED` when the switch is subsequently released.
     ****************************************************************************/
    [[deprecated("Use update(delayTime) instead.")]]
    uint8_t DetectLongPress(const uint32_t longPressTime = 1000);

    /****************************************************************************
     * Evaluates a previously read switch state for a long press.
     *
     * **Deprecated** — long-press detection is now handled directly inside
     * `update()` via its `delayTime` parameter. Prefer using `update(delayTime)`.
     *
     * @param buttonState   A switch-state bitmask returned by a prior call to
     *                      `Read()` or `update()`.
     * @param longPressTime Milliseconds the switch must be held before a
     *                      long-press is reported. Defaults to 2000 ms.
     * @return A bitmask composed of the switch-state constants. Returns
     *         `LONG_PRESS_PRESSED` when a long press is first detected, and
     *         `LONG_PRESS_RELEASED` when the switch is subsequently released.
     *         Returns `buttonState` unchanged if no long-press condition is met.
     ****************************************************************************/
    [[deprecated("Use update(delayTime) instead.")]]
    uint8_t DetectLongPress(uint8_t buttonState, const uint32_t longPressTime = 2000);

    /****************************************************************************
     * `TaskBase` polling callback — called automatically by the task scheduler.
     *
     * Delegates to `update()` with default arguments to perform a debounced read
     * and fire `StateChanged` on any state transition.
     *
     * @return void
     ****************************************************************************/
    void Poll() override;
    
    /****************************************************************************
     * Reads the current switch state with optional debouncing and repeat/long-
     * press detection.
     *
     * **Deprecated** — Use `update()` instead.
     *
     * @param delayTime  Milliseconds before a long-press or first auto-repeat
     *                   event is generated. `0` disables both features.
     * @param repeatTime Milliseconds between subsequent auto-repeat events.
     *                   `0` disables auto-repeat (long-press mode only).
     * @return A bitmask composed of the switch-state constants (`OFF`, `ON`,
     *         `PRESSED`, `RELEASED`, `LONG_PRESS_PRESSED`, etc.).     
     ****************************************************************************/
    [[deprecated("Use update() instead.")]]
    uint8_t Read(const uint32_t delayTime=0, const uint32_t repeatTime=0)
    {
        return update(delayTime, repeatTime);
    };
    
    /****************************************************************************
     * Returns the current state of the switch as of the last call to `update()`.
     *
     * @return A bitmask composed of the switch-state constants (`OFF`, `ON`,
     *         `PRESSED`, `RELEASED`, `LONG_PRESS_PRESSED`, etc.).     
     ****************************************************************************/
    inline uint8_t getState() 
    {
        auto switchState = state.currentState | (state.longPress ? LONG_PRESS : 0)
                                              | (state.toggled ? TOGGLED : 0);
        
        return switchState; 
    };

    //****************************************************************************
    // Internal State
    //****************************************************************************
private:
    struct // Occupies 1 byte
    {
        uint8_t activeLow        : 1; // Indicates if the switch is active (ON) on LOW reading
        uint8_t currentState     : 1; // Current switch state (1=ON, 0=OFF)
        uint8_t toggled          : 1; // Indicates if the switch was toggled on the last call to update()
        uint8_t longPress        : 1; // Indicates a long press was detected.
        uint8_t repeating        : 1; // Indicates a repeat press is in progress.
    }
    state;
    
    uint8_t  pin;               // Digital pin the switch is connected to
    float    debounceFilter;    // The last computed value of the debounce filter
    uint32_t lastPressTime;     // Time the last button press occurred (after debouncing)

private:
    // Reads the switch input pin and inverts the sense if active low so that a 
    // return value of 1 always means the switch is activated (ON) and 0 means 
    // the switch is inactivated (OFF).
    inline uint8_t readPin()
    {
        uint8_t pinState = digitalRead(pin);

        return state.activeLow ? !pinState : pinState;
    }
};

#endif
