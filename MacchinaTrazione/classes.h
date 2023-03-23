#include <stdint.h>
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
#include <Arduino.h>
//#include <esp_task_wdt.h>
#include "HX711.h"


#define CMD_SERIAL_LENGTH 8  // max message length
#define CMD_MATLAB_LENGTH 7  // matlab message length
#define CMD_MATLAB_OUTPUT 12  // matlab output length
#define CMD_MATLAB_START 99  
#define CMD_MATLAB_ACK 98
#define SERIAL_MANUAL 88 // ASCII X character

#define MODE_MANUAL 0
#define MODE_CAL_UP 1
#define MODE_CAL_DOWN 2
#define MODE_WEIGHT 3
#define MODE_LENGTH 4
#define MODE_TESTEND 9  // this mode is used to signal matlab the test has ended

#define MOVE_INDEF 0

#define LED_OFF 0
#define LED_ERROR 1
#define LED_OK 2
#define LED_1 3
#define LED_2 4
#define LED_12 5
#define LED_3 6
#define LED_AUTO_SELECT 7
#define LED_AUTO_UP 8
#define LED_AUTO_DOWN 9

#define MANUAL_FREQ 50  // millis cooldown between measurements
#define CAL_TIME 1500  // millis time to press to start calibration

#define SPEED_STEPS 4
#define DELAY_MIN 1000
#define SPEED_1 2 // speeds in mm/min
#define SPEED_2 5
#define SPEED_3 20
#define SPEED_4 55 // this is max speed with delay 1ms

// input configuration
#define PIN_ENDSTOP_1 19
#define PIN_ENDSTOP_2 21
#define PIN_ENDSTOP_3 22
#define PIN_ENDSTOP_4 23
//#define PIN_MANUAL 5
//#define PIN_CONFIRM 6
#define PIN_SPEED 35
#define PIN_UP 39
#define PIN_DOWN 36
#define PIN_ALT 34
// output configuration
#define PIN_STEPA 17  // step A enable
#define PIN_STEPB 5  // step B enable
#define PIN_STEP 18 // step command
#define PIN_DIR 16 // do not need 2 direction pins as asymmetric movement is not planned
#define PIN_LED1 2 // green top led
#define PIN_LED2 4 // green bottom led
#define PIN_LED3 15 // red led

struct reading
{
  uint8_t mode;
  uint16_t speed; // in 0.01 mm/min
  uint32_t timestamp; // millis
  uint16_t force; // in 0.1N
  uint16_t ex1; // in microm
};

class frame
{
  private:
    uint8_t _mode;
    uint8_t _step_active; // 0 both, 1 A, 2 B
    uint32_t _step_delay; // micros
    float _speed; // mm/min
    bool _dir;  // true is up, false is down
    bool _space_up;
    bool _space_down;
  public:
    void init();
    void up(uint32_t steps);
    void up();
    void down(uint32_t steps);
    void down();
    void stop();
    void checkLimit();

    bool setMode(uint8_t mode_new);
    void setSteppers(uint8_t steppers);
    void setDelay(uint32_t delay);
    void setSpeed(uint16_t speed);
    void setSpeedInt(uint8_t speed_int);

    uint8_t getMode();
    uint8_t getSteppers();
    uint32_t getDelay();
    float getSpeed();
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

void step();
void teststart();
void testend();
void ledcmd(uint8_t code);
float cmdtoi(char *cmd);




extern input endstops[];

extern float loadcell_hz;

extern struct reading reading_last;
extern float force_current;  // value in N
extern float force_target;
extern float ex1_current;  // value in ?
extern float ex1_target;
extern float move_target_microm;

extern uint16_t millis_elapsed;

extern frame mainframe;
extern uint8_t pulse_length;
extern float steps_per_mm; // Steps per rev * Microstepping * Gear reduction ratio / Pitch
extern float steps_per_microm;
extern uint8_t encoder_speed;
extern bool speed_read_encoder;
extern uint32_t test_millis_start;
extern uint32_t test_steps;

extern bool serial_matlab;



extern TaskHandle_t RTOS_stepControl_handle;
extern QueueHandle_t RTOS_stepControl_queue;
extern TaskHandle_t RTOS_modeManager_handle;
extern QueueHandle_t RTOS_modeManager_queue;
extern TaskHandle_t RTOS_ledManager_handle;
extern QueueHandle_t RTOS_ledManager_queue;
extern TaskHandle_t RTOS_serialComm_handle;
extern TaskHandle_t RTOS_dataReader_handle;
extern QueueHandle_t RTOS_readings_queue;
