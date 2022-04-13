#include <stdint.h>
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
#include <Arduino.h>



class frame
{
  private:
    uint8_t _mode;
    uint8_t _step_active; // 0 both, 1 A, 2 B
    uint16_t _step_delay;
    bool _dir;  // true is up, false is down
    //bool _stop;
    bool _space_up;
    bool _space_down;
  public:
    frame();

    void up();
    void down();
    void stop();
    void checkLimit();

    bool setMode(uint8_t mode_new);
    void setSteppers(uint8_t steppers);
    void setDelay(uint16_t delay);

    uint8_t getMode();
    uint8_t getSteppers();
    uint16_t getDelay();
    bool getDir();
    bool cgUp(); // can go up
    bool cgDown(); // can go down
};

class input
{
  private:
    uint8_t _pin;
    bool _state;
    bool _state_pressed;
    bool _justPressed;
    bool _justReleased;

  public:
    input(uint8_t pin, bool state);

    void read();

    bool isPressed();
    bool isReleased();
    bool justPressed();
    bool justReleased();
};

class led
{
  private:
    uint8_t _pin;
    bool _state;

  public:
    led(uint8_t pin);

    void on();
    void off();
};

extern input endstops[];

extern TaskHandle_t RTOS_stepControl_handle;
extern TaskHandle_t RTOS_modeManager_handle;
extern QueueHandle_t RTOS_modeManager_queue;
