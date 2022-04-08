#include "classes.h"

#define SPEED_STEPS 3
#define DELAY_1 50
#define DELAY_2 100
#define DELAY_3 200

// input variables
uint8_t pin_manual;
uint8_t pin_confirm;
uint8_t pin_speed;
uint8_t pin_up;
uint8_t pin_down;
//input btn_manual;
//input btn_confirm;
uint8_t encoder_speed;
input btn_up(pin_up, true);
input btn_down(pin_down, true);


// stepper variables
uint8_t pin_stepA;
uint8_t pin_stepB;
uint8_t pin_dir;  // do not need 2 direction pins as asymmetric movement is not planned
uint8_t pulse_length;

frame mainframe;

TaskHandle_t RTOS_stepControl_handle;
TaskHandle_t RTOS_modeManager_handle;
QueueHandle_t RTOS_modeManager_queue;


void setup() {
  pinMode(pin_stepA, OUTPUT);
  pinMode(pin_stepB, OUTPUT);
  pinMode(pin_dir, OUTPUT);
  pinMode(pin_manual, INPUT);
  pinMode(pin_confirm, INPUT);
  pinMode(pin_speed, INPUT);
  // put your setup code here, to run once:

  // xTaskCreate(
  //  time_display_update,  // Function that should be called
  //  "Time display update",   // Name of the task (for debugging)
  //  1000,     // Stack size (bytes)
  //  NULL,     // Parameter to pass
  //  1,         // Task priority
  //  &time_display_update_handle      // Task handle
  //  );


  RTOS_modeManager_queue = xQueueCreate(4, sizeof(uint8_t));
  xTaskCreate(stepControl, "Step controller", 3000, NULL, 2, &RTOS_stepControl_handle);
}

// loop has idle task hook, it runs when nothing else is going on
void loop() {

}


void updateInput()
{
  //btn_manual = digitalRead(pin_manual);
  //btn_confirm = digitalRead(pin_confirm);
  //encoder_speed = analogRead(pin_speed);
  //btn_up = digitalRead(pin_up);
  //btn_down = digitalRead(pin_down);
}


void stepControl(void * parameter)
{
  bool dir;
  uint8_t step_active; // 0 both, 1 A, 2 B
  uint16_t step_delay;

  for (;;)
  {
    //Serial.println(uxTaskGetStackHighWaterMark( NULL ));
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    // get movement variables
    dir = mainframe.getDir();
    step_active = mainframe.getSteppers();

    // set steppers direction
    if (dir)  // check if dir is reversed
    {
      digitalWrite(pin_dir, HIGH);
    }
    else
    {
      digitalWrite(pin_dir, LOW);
    }

    // move correct steppers
    switch (step_active)
    {
      case 0: // both steppers active
      {
        while (!mainframe.isStopped())
        {
          step_delay = mainframe.getDelay();  // this allows for changing without releasing movement button
          // NOTE: digitalWrite funtion is not fast to execute, but the delay should be << compared to pulse_lenght
          // this means there is a slight asymmetry and delay between the right and left steppers, but it should not be an issue
          // TODO: check if it's really an issue or not
          digitalWrite(pin_stepA, HIGH);
          digitalWrite(pin_stepB, HIGH);
          delayMicroseconds(pulse_length);
          digitalWrite(pin_stepA, LOW);
          digitalWrite(pin_stepB, LOW);
          delayMicroseconds(step_delay);
        }
      }
      case 1: // stepper A active
      {
        while (!mainframe.isStopped())
        {
          step_delay = mainframe.getDelay();
          digitalWrite(pin_stepA, HIGH);
          delayMicroseconds(pulse_length);
          digitalWrite(pin_stepA, LOW);
          delayMicroseconds(step_delay);
        }
      }
      case 2: // stepper B active
      {
        while (!mainframe.isStopped())
        {
          step_delay = mainframe.getDelay();
          digitalWrite(pin_stepB, HIGH);
          delayMicroseconds(pulse_length);
          digitalWrite(pin_stepB, LOW);
          delayMicroseconds(step_delay);
        }
      }
    }
  }
}

void modeControl(void * parameter)
{
  uint8_t mode;
  for (;;)
  {
    // queue mode
    mode = mainframe.getMode();
      switch (mode)
      {
        case 0:
          {
            
          }
          break;

      }

  }
}

void modeManager(void * parameter)
{
  uint8_t mode;
  for (;;)
  {
    if (xQueueReceive(RTOS_modeManager_queue, &mode, 0) == pdTRUE)  // pdTRUE means there is a mode in queue
    switch (mode)
    {
      // automatic modes 
    }
    else  // pdFALSE, there is no mode in queue so we are in manual mode
    {
      // read buttons
      btn_up.read();
      btn_down.read();
      // calculate speed
      encoder_speed = map(analogRead(pin_speed), 0, 4095, 1, SPEED_STEPS);
      uint16_t delay_new;
      switch (encoder_speed)
      {
        case 1:
          delay_new = DELAY_1;
          break;
        case 2:
          delay_new = DELAY_2;
          break;
        case 3:
          delay_new = DELAY_3;
          break;
      }
      if (delay_new != mainframe.getDelay())
      {
        mainframe.setDelay(delay_new);
      }

      // select movement
      if (btn_up.isPressed() && btn_down.isPressed()) // should not press both, failsafe stops
      {
        mainframe.stop();
      }
      else
      {
        if (btn_up.justPressed())
        {
          mainframe.stop();
          mainframe.up();
        }
        else if (btn_down.justPressed())
        {
          mainframe.stop();
          mainframe.down();
        }
        else if (btn_up.justReleased() || btn_down.justReleased())
        {
          mainframe.stop();
        }  
      }
      vTaskDelay(50); // check every 50 ms
    }
  }
}