#include "classes.h"

//input btn_manual;
//input btn_confirm;
uint16_t encoder_speed = 1000;
// input endstops[4] = { input(PIN_ENDSTOP_1, true), input(PIN_ENDSTOP_2, true), input(PIN_ENDSTOP_3, true), input(PIN_ENDSTOP_4, true)};
input endstops[4] = {{PIN_ENDSTOP_1, false}, {PIN_ENDSTOP_2, false}, {PIN_ENDSTOP_3, false}, {PIN_ENDSTOP_4, false}};
// input btn_confirm(PIN_CONFIRM, false);
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
uint8_t pulse_length = 10;

frame mainframe;

TaskHandle_t RTOS_stepControl_handle;
QueueHandle_t RTOS_stepControl_queue;
// this semaphore taken means red: stop the movement
// if semaphore free movement can happen 
//SemaphoreHandle_t RTOS_stepControl_semaphore;
TaskHandle_t RTOS_modeManager_handle;
QueueHandle_t RTOS_modeManager_queue;
TaskHandle_t RTOS_ledManager_handle;
QueueHandle_t RTOS_ledManager_queue;


// void IRAM_ATTR endstop_trigger()
// {
//   mainframe.checkLimit();
// }


void setup()
{
  Serial.begin(115200);
  Serial.println("Booting...");
  RTOS_ledManager_queue = xQueueCreate(4, sizeof(uint8_t));
  xTaskCreate(ledManager, "Led manager", 3000, NULL, 2, &RTOS_ledManager_handle);
  ledMessage(LED_ERROR);
  pinMode(PIN_STEPA, OUTPUT);
  pinMode(PIN_STEPB, OUTPUT);
  pinMode(PIN_DIR, OUTPUT);
  // pinMode(PIN_SPEED, INPUT);
  // attachInterrupt(PIN_ENDSTOP_1, endstop_trigger, CHANGE);
  // attachInterrupt(PIN_ENDSTOP_2, endstop_trigger, CHANGE);
  // attachInterrupt(PIN_ENDSTOP_3, endstop_trigger, CHANGE);
  // attachInterrupt(PIN_ENDSTOP_4, endstop_trigger, CHANGE);
  // put your setup code here, to run once:

  // xTaskCreate(
  //  time_display_update,  // Function that should be called
  //  "Time display update",   // Name of the task (for debugging)
  //  1000,     // Stack size (bytes)
  //  NULL,     // Parameter to pass
  //  1,         // Task priority
  //  &time_display_update_handle      // Task handle
  //  );
  //RTOS_stepControl_semaphore = xSemaphoreCreateBinary();
  //xSemaphoreGive(RTOS_stepControl_semaphore);
  RTOS_modeManager_queue = xQueueCreate(4, sizeof(uint8_t));
  RTOS_stepControl_queue = xQueueCreate(4, sizeof(uint8_t));

  xTaskCreate(stepControl, "Step controller", 3000, NULL, 10, &RTOS_stepControl_handle);
  xTaskCreate(modeManager, "Mode manager", 3000, NULL, 3, &RTOS_modeManager_handle);

  Serial.print("Mainframe initalization...");
  mainframe.init();
  Serial.println(" done!");

  // notify setup ended
  Serial.println("All done! Hello.");
  ledMessage(LED_OK);
}


// loop has idle task hook, it runs when nothing else is going on
// left empty to allow resource cleanup
void loop() {}

void ledMessage(uint8_t code)
{
  xTaskNotifyGive(RTOS_ledManager_handle);  // clears any previous message
  xQueueSend(RTOS_ledManager_queue, &code, 0);
}

void stepControl(void * parameter)
{
  uint8_t move_mode;
  bool dir;
  uint8_t step_active; // 0 both, 1 A, 2 B
  uint16_t step_delay;

  for (;;)
  {
    // we are using a queue to allow for different moving modes in future
    // aslo because multiple notification to task are NOT working (don't know why)
    xQueueReceive(RTOS_stepControl_queue, &move_mode, portMAX_DELAY);
    ulTaskNotifyTake(pdTRUE, 0);  // clear previous notifications

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
    // stopping will be handled by interrupt, we just want to now if we can start moving
    if (direction_allowed)
    {
      switch (move_mode)
      {
        case MOVE_INDEF:
        {
          // ulTaskNotifyTake(pdTRUE, 0) returns notification value if there are notifications
          // ulTaskNotifyTake(pdTRUE, 0) returns 0 if there are no notifications
          // so !ulTaskNotifyTake(pdTRUE, 0) returns 1 if there are no notifications
          while (!ulTaskNotifyTake(pdTRUE, 0))  // do not wait if there is no notification
          {
            // we can move
            step_delay = mainframe.getDelay();  // this allows for changing speed without releasing movement button
            switch (step_active)
            {
              case 0: // both steppers active
              { 
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
              case 1: // stepper A active
              {
                digitalWrite(PIN_STEPA, HIGH);
                delayMicroseconds(pulse_length);
                digitalWrite(PIN_STEPA, LOW);
                delayMicroseconds(step_delay);
              }
              case 2: // stepper B active
              {
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
  }
}

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
              ledMessage(LED_CAL_UP);
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
              ledMessage(LED_OK);
            }
            break;
          case MODE_CAL_DOWN:
            {
              ledMessage(LED_CAL_DOWN);
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
              ledMessage(LED_OK);
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
      //encoder_speed = map(analogRead(PIN_SPEED), 0, 4095, 1, SPEED_STEPS);
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
          ledMessage(LED_12);
          // pressing both buttons for 5s also starts the calibration routine
          // this method is not really accurate but it doesn't have to be
          // this i because it takes > vTaskDelay time to execute all the code
          millis_elapsed = 0;
        }
        else  // buttons were already both pressed
        {
          millis_elapsed += MANUAL_FREQ;
          // this selects and starts calibration routine
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
        if (btn_up.justPressed())// && endstops[0].isReleased() && endstops[1].isReleased()) // check movement bounds
        {
          mainframe.stop();
          mainframe.up();
          ledMessage(LED_1);
        }
        else if (btn_down.justPressed())// && endstops[2].isReleased() && endstops[3].isReleased())
        {
          mainframe.stop();
          mainframe.down();
          ledMessage(LED_2);
        }
        else if (btn_up.justReleased() || btn_down.justReleased())
        {
          mainframe.stop();
          ledMessage(LED_OFF);
        }  
      }

      // check if calibration must start
      vTaskDelay(MANUAL_FREQ); // check every MANUAL_FREQ ms
    }
  }
}


void ledManager(void * parameter)
{
  uint8_t message;
  for (;;)
  {
    xQueueReceive(RTOS_ledManager_queue, &message, portMAX_DELAY);
    ulTaskNotifyTake(pdTRUE, 0);  // clear previous notifications
    Serial.println((String)"Led: "+message);

    switch (message)
    {
      case LED_OFF:
        {
          led1.off();
        }
        break;
      case LED_ERROR:
        {
          led3.on();
          vTaskDelay(2000);
          Serial.print("Waiting for notify...");
          ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // keep led on if code has an issue and doesn't unlock
          Serial.println(" go!");
          led1.on();
          led2.on();
          vTaskDelay(2000);
          led1.off();
          led2.off();
          led3.off();
          vTaskDelay(200);
        }
        break;
      case LED_OK:
        {
          for (uint8_t i = 0; i < 5; ++i)
          {
            led1.on();
            led2.on();
            vTaskDelay(100);
            led1.off();
            led2.off();
            vTaskDelay(100);
          }
        }
        break;
      case LED_1:
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
      case LED_2:
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
      case LED_12:
        {
          led1.on();
          led2.on();
          while (!ulTaskNotifyTake(pdTRUE, 0))
          {
            vTaskDelay(10);
          }
          led1.off();
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
      case LED_CAL_UP:
        {
          led1.on();
          led3.on();
          while (!ulTaskNotifyTake(pdTRUE, 0))
          {
            vTaskDelay(10);
          }
          led1.off();
          led3.off();
          vTaskDelay(200);
        }
        break;
      case LED_CAL_DOWN:
        {
          led2.on();
          led3.on();
          while (!ulTaskNotifyTake(pdTRUE, 0))
          {
            vTaskDelay(10);
          }
          led2.off();
          led3.off();
          vTaskDelay(200);
        }
        break;
    }
  }
}