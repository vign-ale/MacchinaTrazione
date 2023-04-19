#include "definitions.h"

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

void frame::init()
{
  setMode(0);  // start in manual mode
  setSteppers(0);
}

void frame::up(uint32_t microm)
{
  _dir = true;
  uint32_t move_step = microm * steps_per_microm;
  xQueueSend(RTOS_stepControl_queue, &move_step, 0);
}

void frame::up()
{
  up(MOVE_INDEF);
}

void frame::down(uint32_t microm)
{
  _dir = false;
  uint32_t move_step = microm * steps_per_microm;
  xQueueSend(RTOS_stepControl_queue, &move_step, 0);
}

void frame::down()
{
  down(MOVE_INDEF);
}

void frame::stop()
{
  xTaskNotifyGive(RTOS_stepControl_handle);
}

void frame::checkLimit()
{
  for (uint8_t i = 0; i < 4; ++i)
  {
    endstops[i].read();
    // if (endstops[i].isPressed())
    // {
    //   stop(); //  endstop changed to pressed, stop frame
    // }
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
  if ((_dir && _space_up == false) || (!_dir && _space_down == false))
  {
    stop();
  }
}

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

void frame::setDelay(uint32_t step_delay_new)
{
  if (step_delay_new == 0)
  {
    speed_read_encoder = true;
  }
  else
  {
    if (step_delay_new != _step_delay)
    {
      _step_delay = step_delay_new;
      uint32_t speed_step_delay = _step_delay + pulse_length;
      float speed_new = ((float) 1000000 / speed_step_delay) * ((float) 60 / steps_per_mm);
      _speed = speed_new;
      if (!serial_matlab) Serial.println((String)"Delay stepper: "+_step_delay+" Speed: "+_speed);
    }
  }
}

void frame::setSpeed(float speed)
{
  // sending speed 0 enables manual speed control
  // input speed is in mm/min
  float steps_second = speed * steps_per_mm / 60;  // conversion to s from min
  if (!serial_matlab) Serial.println((String)"Steps per second: "+steps_second);

  float delay_micros = 0;
  if (steps_second != 0)
  {
    delay_micros = (float) 1000000 / steps_second;  // (micros in s) / (steps each second)
    delay_micros = delay_micros - pulse_length; // additional delay from pulse_length removed
    delay_micros = round(delay_micros); // return int delay

    // prevents negative values from causing errors
    
  }
  if (delay_micros >= DELAY_MIN || delay_micros == 0)
  {
    if (!serial_matlab) Serial.println((String)"Setting speed, delay requested: "+delay_micros);
    setDelay(delay_micros);
  }
  else if (delay_micros > 0)
  {
    delay_micros = DELAY_MIN;
    if (!serial_matlab) Serial.println((String)"Setting speed to minimum delay: "+delay_micros);
    setDelay(delay_micros);
  }
  else
  {
    if (!serial_matlab) Serial.println((String)"Invalid speed!");
  }
}

void frame::setSpeedInt(uint8_t speed_int)
{
  float speed_new;
  switch (speed_int)
  {
    case 0:
      speed_new = 0;
    case 1:
      speed_new = SPEED_1;
      break;
    case 2:
      speed_new = SPEED_2;
      break;
    case 3:
      speed_new = SPEED_3;
      break;
    case 4:
      speed_new = SPEED_4;
      break;
    default:
      speed_new = SPEED_1;
  }
  setSpeed(speed_new);
}

uint8_t frame::getMode()
{
  return _mode;
}

uint8_t frame::getSteppers()
{
  return _step_active;
}

uint32_t frame::getDelay()
{
  return _step_delay;
}

float frame::getSpeed()
{
  return _speed;
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
  if (_state_pressed)
  {
    pinMode(_pin, INPUT_PULLDOWN);
  }
  else
  {
    pinMode(_pin, INPUT_PULLUP);
  }
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
      //if (!serial_matlab) Serial.println("Input pressed");
    }
    _state = true;
  }
  else
  {
    if (_state == true)
    {
      _justReleased = true;
      //if (!serial_matlab) Serial.println("Input released");
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
