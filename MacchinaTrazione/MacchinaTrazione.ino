#include "classes.h"


#define MODE_MANUAL 0
#define MODE_CAL 1

// configure which side to use for calibration
#define CAL_SIDE 0  // 0 top, 1 bottom

#define SPEED_STEPS 3
#define DELAY_1 50
#define DELAY_2 200
#define DELAY_3 500
#define DELAY_4 1000

// input configuration
#define PIN_ENDSTOP_1 1
#define PIN_ENDSTOP_2 2
#define PIN_ENDSTOP_3 3
#define PIN_ENDSTOP_4 4
#define PIN_MANUAL 5
#define PIN_CONFIRM 6
#define PIN_SPEED 7
#define PIN_UP 8
#define PIN_DOWN 9
//input btn_manual;
//input btn_confirm;
uint8_t encoder_speed;
//input endstops[4] = { input(PIN_ENDSTOP_1, true), input(PIN_ENDSTOP_2, true), input(PIN_ENDSTOP_3, true), input(PIN_ENDSTOP_4, true)};
input endstops[4] = {{PIN_ENDSTOP_1, false,}, {PIN_ENDSTOP_2, false}, {PIN_ENDSTOP_3, false}, {PIN_ENDSTOP_4, false}};
input btn_confirm(PIN_CONFIRM, false);
input btn_up(PIN_UP, false);
input btn_down(PIN_DOWN, false);


// stepper configuration
#define PIN_STEPA 10
#define PIN_STEPB 11
#define PIN_DIR 12 // do not need 2 direction pins as asymmetric movement is not planned
uint8_t pulse_length;

frame mainframe;

TaskHandle_t RTOS_stepControl_handle;
TaskHandle_t RTOS_modeManager_handle;
QueueHandle_t RTOS_modeManager_queue;
//SemaphoreHandle_t RTOS_stop_semaphore;


void IRAM_ATTR endstop_trigger()
{
  mainframe.checkLimit();
}


void setup() {
  Serial.begin(9600);
  pinMode(PIN_STEPA, OUTPUT);
  pinMode(PIN_STEPB, OUTPUT);
  pinMode(PIN_DIR, OUTPUT);
  pinMode(PIN_SPEED, INPUT);
  attachInterrupt(PIN_ENDSTOP_1, endstop_trigger, CHANGE);
  attachInterrupt(PIN_ENDSTOP_2, endstop_trigger, CHANGE);
  attachInterrupt(PIN_ENDSTOP_3, endstop_trigger, CHANGE);
  attachInterrupt(PIN_ENDSTOP_4, endstop_trigger, CHANGE);
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
  //RTOS_stop_semaphore = xSemaphoreCreateMutex();
  xTaskCreate(stepControl, "Step controller", 3000, NULL, 10, &RTOS_stepControl_handle);
  xTaskCreate(modeManager, "Mode manager", 3000, NULL, 2, &RTOS_modeManager_handle);
}


// loop has idle task hook, it runs when nothing else is going on
// left empty to allow resource cleanup
void loop() {}


void stepControl(void * parameter)
{
  bool dir;
  uint8_t step_active; // 0 both, 1 A, 2 B
  uint16_t step_delay;

  for (;;)
  {
    //Serial.println(uxTaskGetStackHighWaterMark( NULL ));
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    ulTaskNotifyTakeIndexed(1, pdTRUE, 0);  // clear old stop notifications if present
    // get movement variables
    dir = mainframe.getDir();
    step_active = mainframe.getSteppers();

    // set steppers direction and check if direction is free to move
    bool direction_allowed;
    if (dir)  // dir true is up
    {
      digitalWrite(PIN_DIR, HIGH);
      direction_allowed = mainframe.cgUp();  // free space is up
    }
    else
    {
      digitalWrite(PIN_DIR, LOW);
      direction_allowed = mainframe.cgDown();  // free space is down
    }

    // move correct steppers if there is space to do so
    if (direction_allowed)
    {
      switch (step_active)
      {
        case 0: // both steppers active
        {
          // ulTaskNotifyTakeIndexed(1, pdTRUE, 0) returns 0 if there are no notifications
          // so !ulTaskNotifyTakeIndexed(1, pdTRUE, 0) returns 1 if there are no notifications
          while (!ulTaskNotifyTakeIndexed(1, pdTRUE, 0))  // do not wait if there is no notification
          {
            step_delay = mainframe.getDelay();  // this allows for changing speed without releasing movement button
            // NOTE: digitalWrite funtion is not fast to execute, but the delay should be << compared to pulse_lenght
            // this means there is a slight asymmetry and delay between the right and left steppers, but it should not be an issue
            // TODO: check if it's really an issue or not
            digitalWrite(PIN_STEPA, HIGH);
            digitalWrite(PIN_STEPB, HIGH);
            delayMicroseconds(pulse_length);
            digitalWrite(PIN_STEPA, LOW);
            digitalWrite(PIN_STEPB, LOW);
            delayMicroseconds(step_delay);
          }
        }
        case 1: // stepper A active
        {
          while (!ulTaskNotifyTakeIndexed(1, pdTRUE, 0))
          {
            step_delay = mainframe.getDelay();
            digitalWrite(PIN_STEPA, HIGH);
            delayMicroseconds(pulse_length);
            digitalWrite(PIN_STEPA, LOW);
            delayMicroseconds(step_delay);
          }
        }
        case 2: // stepper B active
        {
          while (!ulTaskNotifyTakeIndexed(1, pdTRUE, 0))
          {
            step_delay = mainframe.getDelay();
            digitalWrite(PIN_STEPB, HIGH);
            delayMicroseconds(pulse_length);
            digitalWrite(PIN_STEPB, LOW);
            delayMicroseconds(step_delay);
          }
        }
      }
    }
  }
}


// void modeControl(void * parameter)
// {
//   uint8_t mode;
//   for (;;)
//   {
//     // queue mode
//     mode = mainframe.getMode();
//       switch (mode)
//       {
//         case 0:
//           {

//           }
//           break;

//       }

//   }
// }


void modeManager(void * parameter)
{
  uint8_t mode;
  for (;;)
  {
    if (xQueueReceive(RTOS_modeManager_queue, &mode, 0) == pdTRUE)  // pdTRUE means there is a mode in queue
    {
      if (mode != MODE_MANUAL)
      {
        mainframe.setMode(mode); // set mainframe mode
        switch (mode)
        {
          case MODE_CAL:
            {
              if (CAL_SIDE == 0)
              {
                mainframe.setSteppers(0);
                mainframe.setDelay(DELAY_1);
                mainframe.up();
                while (mainframe.cgUp())
                {
                  vTaskDelay(10); // check every 10ms if limit is reached
                }
                // now upper limit is reached, an endstop is pressed
                // reach the other limit
                if (endstops[0].isPressed())
                {
                  mainframe.setSteppers(2); // enable B stepper
                }
                else if (endstops[1].isPressed())
                {
                  mainframe.setSteppers(1); // enable A stepper
                }
                else {} // should never get here
                // move the correct stepper
                mainframe.up();
                while (mainframe.cgUp())
                {
                  vTaskDelay(10);
                }
                // now both sides have been pressed
                // slow down, reverse and level slowly
                mainframe.setDelay(DELAY_3);
                mainframe.setSteppers(1);
                mainframe.down();
                while (!mainframe.cgUp())
                {
                  vTaskDelay(1);
                }
                mainframe.stop(); // stop when switch is released
                // now other stepper
                mainframe.setSteppers(2);
                mainframe.down();
                while (!mainframe.cgUp())
                {
                  vTaskDelay(1);
                }
              }
              else if (CAL_SIDE == 1)
              {
                mainframe.setSteppers(0);
                mainframe.setDelay(DELAY_1);
                mainframe.down();
                while (mainframe.cgDown())
                {
                  vTaskDelay(10); // check every 10ms if limit is reached
                }
                // now upper limit is reached, an endstop is pressed
                // reach the other limit
                if (endstops[2].isPressed())
                {
                  mainframe.setSteppers(2); // enable B stepper
                }
                else if (endstops[3].isPressed())
                {
                  mainframe.setSteppers(1); // enable A stepper
                }
                else {} // should never get here
                // move the correct stepper
                mainframe.down();
                while (mainframe.cgDown())
                {
                  vTaskDelay(10);
                }
                // now both sides have been pressed
                // slow down, reverse and level slowly
                mainframe.setDelay(DELAY_3);
                mainframe.setSteppers(1);
                mainframe.up();
                while (!mainframe.cgDown())
                {
                  vTaskDelay(1);
                }
                mainframe.stop(); // stop when switch is released
                // now other stepper
                mainframe.setSteppers(2);
                mainframe.up();
                while (!mainframe.cgDown())
                {
                  vTaskDelay(1);
                }
              }
            }
          // automatic modes
          //default:  // default means we are either in manual or not recognized mode, exit from switch and enter manual mode
        }
        mainframe.setMode(MODE_MANUAL); // always set manual mode after executing automatic mode test
      }
      
    }
    else  // pdFALSE, there is no mode in queue or mode sent is manual so we are in manual mode
    {
      // read buttons
      btn_up.read();
      btn_down.read();
      // calculate speed
      encoder_speed = map(analogRead(PIN_SPEED), 0, 4095, 1, SPEED_STEPS);
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
        Serial.println((String)"Delay stepper: "+delay_new);
      }

      // select movement
      if (btn_up.isPressed() && btn_down.isPressed()) // should not press both, failsafe stops
      {
        mainframe.stop();
      }
      else
      {
        if (btn_up.justPressed() && endstops[0].isReleased() && endstops[1].isReleased()) // check   movement bounds
        {
          mainframe.stop();
          mainframe.up();
        }
        else if (btn_down.justPressed() && endstops[2].isReleased() && endstops[3].isReleased())
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
