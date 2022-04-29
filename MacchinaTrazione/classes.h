#include <stdint.h>
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
#include <Arduino.h>


#define MODE_MANUAL 0
#define MODE_CAL_UP 1
#define MODE_CAL_DOWN 2

#define MOVE_INDEF 0

#define LED_OFF 0
#define LED_ERROR 1
#define LED_OK 2
#define LED_1 3
#define LED_2 4
#define LED_12 5
#define LED_CAL_SELECT 6
#define LED_CAL_UP 7
#define LED_CAL_DOWN 8

#define MANUAL_FREQ 50  // millis cooldown between measurements
#define CAL_TIME 5000  // millis time to press to start calibration

#define SPEED_STEPS 3
#define DELAY_1 10
#define DELAY_2 20
#define DELAY_3 50
#define DELAY_4 100

// input configuration
#define PIN_ENDSTOP_1 34
#define PIN_ENDSTOP_2 35
#define PIN_ENDSTOP_3 36
#define PIN_ENDSTOP_4 39
//#define PIN_MANUAL 5
//#define PIN_CONFIRM 6
//#define PIN_SPEED 7
#define PIN_UP 32
#define PIN_DOWN 33
#define PIN_LED1 14 // green top led
#define PIN_LED2 12 // green bottom led
#define PIN_LED3 13 // red led


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
    void init();
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
extern QueueHandle_t RTOS_stepControl_queue;
extern TaskHandle_t RTOS_modeManager_handle;
extern QueueHandle_t RTOS_modeManager_queue;
