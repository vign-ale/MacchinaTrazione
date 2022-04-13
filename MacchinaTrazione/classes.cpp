#include "classes.h"

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

// FRAME CLASS

frame::frame()
{
  _mode = 0;  // start in manual mode
  // _moving = false;
  // _stop = false;
  _step_active = 0;
  _step_delay = 100;
  checkLimit();
}

void frame::up()
{
  _dir = true;
  xTaskNotifyGive(RTOS_stepControl_handle);
}

void frame::down()
{
  _dir = false;
  xTaskNotifyGive(RTOS_stepControl_handle);
}

void frame::checkLimit()
{
  for (uint8_t i = 0; i < 4; ++i)
  {
    endstops[i].read();
    if (endstops[i].isPressed())
    {
      stop(); //  endstop changed to pressed, stop frame
    }
  }
  switch (_step_active) // allow movement based on selected stepper
  {
    case 0: // both steppers active
      {
        if (endstops[0].isReleased() && endstops[1].isReleased())
        {
          _space_up = true;
        }
        else
        {
          _space_up = false;
        }
        if (endstops[2].isReleased() && endstops[3].isReleased())
        {
          _space_down = true;
        }
        else
        {
          _space_down = false;
        }
      }
      break;
    case 1: // A stepper active
      {
        if (endstops[0].isReleased())
        {
          _space_up = true;
        }
        else
        {
          _space_up = false;
        }
        if (endstops[2].isReleased())
        {
          _space_down = true;
        }
        else
        {
          _space_down = false;
        }
      }
      break;
    case 2: // B stepper active
      {
        if (endstops[1].isReleased())
        {
          _space_up = true;
        }
        else
        {
          _space_up = false;
        }
        if (endstops[3].isReleased())
        {
          _space_down = true;
        }
        else
        {
          _space_down = false;
        }
      }
      break;
  }
}

void frame::stop()
{
  xTaskNotifyGiveIndexed(RTOS_stepControl_handle, 1);
  // _stop = true;
}

// void frame::enable()
// {
//   _stop = false;
// }

bool frame::setMode(uint8_t mode_new) // TODO: notification to task
{
  if (mode_new != _mode)
  {
    _mode = mode_new;
    xQueueSend(RTOS_modeManager_queue, &mode_new, portMAX_DELAY);
    return true;
  }
  return false;
}

void frame::setSteppers(uint8_t steppers)
{
  _step_active = steppers;
  checkLimit();
}

void frame::setDelay(uint16_t step_delay)
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

bool frame::cgUp()
{
  return _space_up;
}

bool frame::cgDown()
{
  return _space_down;
}

// bool frame::isStopped()
// {
//   // TODO: understand if it is really necessary
//   // semaphore is used to make sure that reading and writing to variable is not done simultaneously
//   // xSemaphoreTake(RTOS_stop_semaphore, portMAX_DELAY);
//   // bool temp = _stop;
//   // xSemaphoreGive(RTOS_SPI_semaphore);
//   // return temp;
//   return _stop;
// }


// INPUT CLASS

input::input(uint8_t pin, bool pressed)
{
  _pin = pin;
  _state_pressed = pressed; // change based on HW
  pinMode(_pin, INPUT_PULLUP);
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


// LED CLASS

led::led(uint8_t pin)
{
  _pin = pin;
  _state = false;
  pinMode(_pin, OUTPUT);
  off();
}

void led::on()
{
  _state = true;
  digitalWrite(_pin, HIGH);
}

void led::off()
{
  _state = false;
  digitalWrite(_pin, LOW);
}