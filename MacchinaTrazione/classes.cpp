#include "classes.h"

#define ENDSTOP_PRESSED_STATE true

// class frame
// {
// // private:
//   uint8_t _pin_step;
//   uint8_t _pin_dir;
//   uint8_t _pin_manual;
//   uint8_t _pin_speed;
//   uint8_t _pin_up;
//   uint8_t _pin_down;
//   uint8_t _pin_confirm;
// public:
//   frame();
//   bool setMode(uint8_t mode_new);
//   uint8_t getMode();
//   bool move(int8_t steps);
// }

frame::frame()
{
  _mode = 0;  // start in manual mode
  // _moving = false;
  _stop = false;
  _step_active = 0;
  _step_delay = 100;
}

void frame::up()
{
  if (_stop)
  {
    _dir = true;
    xTaskNotifyGive(RTOS_stepControl_handle);
  }
}

void frame::down()
{
  if (_stop)
  {
    _dir = false;
    xTaskNotifyGive(RTOS_stepControl_handle);
  }
}

void frame::stop()
{
  _stop = true;
}

bool frame::setMode(uint8_t mode_new)
{
  if (mode_new != _mode)
  {
    _mode = mode_new;
    return true;
  }
  return false;
}

void frame::setSteppers(uint8_t steppers)
{
  _step_active = steppers;
}

void frame::setDelay(uint8_t step_delay)
{
  _step_delay = step_delay;
}

uint8_t frame::getMode()
{
  return _mode;
}

uint8_t frame::getSteppers()
{
  return _step_active;
}

uint16_t frame::getDelay()
{
  return _step_delay;
}

bool frame::getDir()
{
  return _dir;
}

bool frame::isStopped()
{
  return _stop;
}


input::input(uint8_t pin, bool pressed)
{
  _pin = pin;
  _state_pressed = pressed; // change based on HW
  pinMode(_pin, INPUT);
  read();
}

void input::read()
{
  // reset just states
  if (_justPressed)
  {
    _justPressed = false;
  }
  if (_justReleased)
  {
    _justReleased = false;
  }
  // read new value
  bool _value = digitalRead(_pin);
  if (_value == _state_pressed)
  {
    if (_state == false)
    {
      _justPressed = true;
    }
    _state = true;
  }
  else
  {
    if (_state == true)
    {
      _justReleased = true;
    }
    _state = false;
  }
}

bool input::isPressed()
{
  return _state;
}

bool input::isReleased()
{
  return !_state;
}

bool input::justPressed()
{
  return _justPressed;
}

bool input::justReleased()
{
  return _justReleased;
}
