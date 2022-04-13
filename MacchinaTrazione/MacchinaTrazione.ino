#include "classes.h"


#define MODE_MANUAL 0
#define MODE_CAL_UP 1
#define MODE_CAL_DOWN 2

#define LED_ERROR 0
#define LED_OK 1
#define LED_UP 2
#define LED_DOWN 3
#define LED_CAL_SELECT 4

#define MANUAL_FREQ 50  // millis cooldown between measurements
#define CAL_TIME 5000  // millis time to press to start calibration

#define SPEED_STEPS 3
#define DELAY_1 10
#define DELAY_2 20
#define DELAY_3 50
#define DELAY_4 100

// input configuration
#define PIN_ENDSTOP_1 1
#define PIN_ENDSTOP_2 2
#define PIN_ENDSTOP_3 3
#define PIN_ENDSTOP_4 4
#define PIN_MANUAL 5
#define PIN_CONFIRM 6
#define PIN_SPEED 7
#define PIN_UP 34
#define PIN_DOWN 35
#define PIN_LED1 9
#define PIN_LED2 10
#define PIN_LED3 11
//input btn_manual;
//input btn_confirm;
uint8_t encoder_speed;
//input endstops[4] = { input(PIN_ENDSTOP_1, true), input(PIN_ENDSTOP_2, true), input(PIN_ENDSTOP_3, true), input(PIN_ENDSTOP_4, true)};
input endstops[4] = {{PIN_ENDSTOP_1, false,}, {PIN_ENDSTOP_2, false}, {PIN_ENDSTOP_3, false}, {PIN_ENDSTOP_4, false}};
input btn_confirm(PIN_CONFIRM, false);
input btn_up(PIN_UP, false);
input btn_down(PIN_DOWN, false);
led led1(PIN_LED1);
led led2(PIN_LED2);
led led3(PIN_LED3);

uint16_t millis_elapsed;


// stepper configuration
#define PIN_STEPA 26
#define PIN_STEPB 25
#define PIN_DIR 27 // do not need 2 direction pins as asymmetric movement is not planned
uint8_t pulse_length;

frame mainframe;

TaskHandle_t RTOS_stepControl_handle;
TaskHandle_t RTOS_modeManager_handle;
QueueHandle_t RTOS_modeManager_queue;
TaskHandle_t RTOS_ledManager_handle;
QueueHandle_t RTOS_ledManager_queue;
//SemaphoreHandle_t RTOS_stop_semaphore;


void IRAM_ATTR endstop_trigger()
{
  mainframe.checkLimit();
}


void setup() {
  Serial.begin(115200);
  Serial.print("Booting...");
  RTOS_ledManager_queue = xQueueCreate(4, sizeof(uint8_t));
  xTaskCreate(ledManager, "Led manager", 3000, NULL, 2, &RTOS_ledManager_handle);
  ledMessage(LED_ERROR);
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
  xTaskCreate(modeManager, "Mode manager", 3000, NULL, 3, &RTOS_modeManager_handle);

  // notify setup ended
  Serial.println("  Hello!");
  ledMessage(LED_OK);
}


// loop has idle task hook, it runs when nothing else is going on
// left empty to allow resource cleanup
void loop() {}

void ledMessage(uint8_t code)
{
  xTaskNotifyGive(RTOS_ledManager_handle);  // clears any previous message
  xQueueSend(RTOS_modeManager_queue, &code, 0);
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
      if (mode != MODE_MANUAL)  // if mode is manual skip, next loop queue is clear and enter manual mode
      {
        mainframe.setMode(mode); // set mainframe mode
        switch (mode)
        {
          case MODE_CAL_UP:
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
            break;
          case MODE_CAL_DOWN:
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
            break;
          // automatic modes
          //default:  // default means we are either in manual or not recognized mode, exit from switch and enter manual mode
        }
        mainframe.setMode(MODE_MANUAL); // always set manual mode after executing automatic mode test
      }
      
    }
    else  // pdFALSE, there is no mode in queue so we are in manual mode
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
      if (btn_up.isPressed() && btn_down.isPressed()) // should not press both, failsafe stops and starts calibration cooldown
      {
        if (btn_up.justPressed() || btn_down.justPressed()) // at least one button has just been pressed
        {
          mainframe.stop();
          // pressing both buttons for 5s also starts the calibration routine
          // this method is not really accurate but we it doesn't have to be
          // this i because it takes > vTaskDelay time to execute all the code
          millis_elapsed = 0;
        }
        else  // buttons were already both pressed
        {
          millis_elapsed += MANUAL_FREQ;
          if (millis_elapsed >= CAL_TIME)
          {
            ledMessage(LED_CAL_SELECT);
            while (btn_up.isPressed() || btn_down.isPressed())  // wait for release of both buttons
            {
              vTaskDelay(MANUAL_FREQ);
              btn_up.read();
              btn_down.read();
            }
            // now both buttons are released
            while (!btn_up.justPressed() && !btn_down.justPressed())  // wait for press of one button
            {
              vTaskDelay(MANUAL_FREQ);
              btn_up.read();
              btn_down.read();
            }
            xTaskNotifyGive(RTOS_ledManager_handle);
            // now a button has been pressed
            if (btn_up.justPressed())
            {
              mainframe.setMode(MODE_CAL_UP);
            }
            else if (btn_down.justPressed())
            {
              mainframe.setMode(MODE_CAL_DOWN);
            }
            else {} // should never get here
          }
        }
      }
      else
      {
        if (btn_up.justPressed() && endstops[0].isReleased() && endstops[1].isReleased()) // check movement bounds
        {
          mainframe.stop();
          mainframe.up();
          ledMessage(LED_UP);
        }
        else if (btn_down.justPressed() && endstops[2].isReleased() && endstops[3].isReleased())
        {
          mainframe.stop();
          mainframe.down();
          ledMessage(LED_DOWN);
        }
        else if (btn_up.justReleased() || btn_down.justReleased())
        {
          mainframe.stop();
          xTaskNotifyGive(RTOS_ledManager_handle);
        }  
      }

      // check if calibration must start
      vTaskDelay(MANUAL_FREQ); // check every 50 ms
    }
  }
}


void ledManager(void * parameter)
{
  uint8_t message;
  for (;;)
  {
    xQueueReceive(RTOS_ledManager_queue, &message, portMAX_DELAY);
    switch (message)
    {
      case LED_ERROR:
        {
          led3.on();
          vTaskDelay(2000);
          ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // keep led on if code has an issue and doesn't unlock
          led3.off();
          led1.on();
          led2.on();
          vTaskDelay(2000);
          led1.off();
          led2.off();
          vTaskDelay(200);
        }
        break;
      case LED_OK:
        {
          for (uint8_t i = 0; i < 3; ++i)
          {
            led1.on();
            led2.on();
            vTaskDelay(200);
            led1.off();
            led2.off();
            vTaskDelay(200);
          }
        }
        break;
      case LED_UP:
        {
          led1.on();
          while (!ulTaskNotifyTake(pdTRUE, 0))
          {
            vTaskDelay(10);
          }
          led1.off();
          vTaskDelay(200);
        }
        break;
      case LED_DOWN:
        {
          led2.on();
          while (!ulTaskNotifyTake(pdTRUE, 0))
          {
            vTaskDelay(10);
          }
          led2.off();
          vTaskDelay(200);
        }
        break;
      case LED_CAL_SELECT:
        {
          while (!ulTaskNotifyTake(pdTRUE, 0))
          {
            vTaskDelay(200);
            led1.on();
            led2.off();
            vTaskDelay(200);
            led1.off();
            led2.on();
          }
          led2.off();
          vTaskDelay(200);
        }
        break;
    }
  }
}
