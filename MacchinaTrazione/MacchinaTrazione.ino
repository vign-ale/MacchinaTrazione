#include "classes.h"


input endstops[4] = {{PIN_ENDSTOP_1, false}, {PIN_ENDSTOP_2, false}, {PIN_ENDSTOP_3, false}, {PIN_ENDSTOP_4, false}};
input btn_up(PIN_UP, false);
input btn_down(PIN_DOWN, false);
led led1(PIN_LED1);
led led2(PIN_LED2);
led led3(PIN_LED3);

HX711 loadcell, ex1;
#define PIN_LOADCELL_DOUT 32
#define PIN_LOADCELL_SCK 33
#define PIN_EX1_DOUT 25
#define PIN_EX1_SCK 26
const uint32_t LOADCELL_OFFSET = 50682624;  // tare raw value
const uint32_t LOADCELL_DIVIDER = 5895655;  // raw to N conversion value
float loadcell_hz = 1;

struct reading reading_last;
float force_current = 0;  // value in 0.1N
float force_target = 0;
float ex1_current = 0;  // value in ?
float ex1_target = 0;
float move_target_microm = 1000;

uint16_t millis_elapsed;

// stepper configuration
frame mainframe;
uint8_t pulse_length = 10;
float steps_per_mm = 200 * 1 * 26.85 / 5; // Steps per rev * Microstepping * Gear reduction ratio / Pitch
float steps_per_microm = 200 * 1 * 26.85 / 5000; // Steps per rev * Microstepping * Gear reduction ratio / Pitch
uint8_t encoder_speed = 1;
bool speed_read_encoder = true;
uint32_t test_millis_start = 0;
uint32_t test_steps = 0;

// serial configuration
bool serial_matlab = false;


// RTOS stuff
TaskHandle_t RTOS_stepControl_handle;
QueueHandle_t RTOS_stepControl_queue;
TaskHandle_t RTOS_modeManager_handle;
QueueHandle_t RTOS_modeManager_queue;
TaskHandle_t RTOS_ledManager_handle;
QueueHandle_t RTOS_ledManager_queue;
TaskHandle_t RTOS_serialComm_handle;
TaskHandle_t RTOS_dataReader_handle;
QueueHandle_t RTOS_readings_queue;


// DEPRECATED

//TaskHandle_t RTOS_limitCheck_handle;

// here we use an ISR to activate a task that runs checkLimit
// the task is not used elsewhere as it is better to make it run in other task
// this way we are always sure the checkLimit ended before anything else happens in the task
// in this case we come from an ISR so the task has really high priority and will be run ASAP
// void IRAM_ATTR endstop_trigger()
// {
//   // use xHigherPriorityTaskWoken to make sure context switch is as fast as possible
//   BaseType_t xHigherPriorityTaskWoken = pdFALSE;
//   vTaskNotifyGiveFromISR(RTOS_limitCheck_handle, &xHigherPriorityTaskWoken);
//   portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
// }

// void limitCheck(void * parameter)
// {
//   for (;;)
//   {
//     ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
//     mainframe.checkLimit();
//   }
// }

void IRAM_ATTR endstop_trigger()
{
  mainframe.checkLimit();
}

void IRAM_ATTR emergency_trigger()
{
  // this means we have called and abort command
  // restarting ESP is the only way to make sure we terminate every task
  ets_printf("EMERGENCY STOP!\nRebooting, see you soon!\n");
  ESP.restart();
}

void setup()
{
  Serial.begin(115200);
  Serial.println("\n\nBooting...");
  RTOS_ledManager_queue = xQueueCreate(4, sizeof(uint8_t));
  xTaskCreate(ledManager, "Led manager", 3000, NULL, 2, &RTOS_ledManager_handle);
  ledcmd(LED_ERROR);
  
  vTaskDelay(500);
  // pinMode(PIN_SPEED, INPUT); not sure why not needed?
  
  // put your setup code here, to run once:
  // xTaskCreate(
  //  time_display_update,  // Function that should be called
  //  "Time display update",   // Name of the task (for debugging)
  //   1000,     // Stack size (bytes)
  //  NULL,     // Parameter to pass
  //  1,         // Task priority
  //  &time_display_update_handle      // Task handle
  //  );

  Serial.print("Mainframe initalization...\n");
  pinMode(PIN_STEP, OUTPUT);
  pinMode(PIN_STEPA, OUTPUT);
  pinMode(PIN_STEPB, OUTPUT);
  pinMode(PIN_DIR, OUTPUT);
  pinMode(PIN_ALT, INPUT);
  pinMode(PIN_ENDSTOP_1, INPUT);
  pinMode(PIN_ENDSTOP_2, INPUT);
  pinMode(PIN_ENDSTOP_3, INPUT);
  pinMode(PIN_ENDSTOP_4, INPUT);
  mainframe.init();

  loadcell.begin(PIN_LOADCELL_DOUT, PIN_LOADCELL_SCK);
  loadcell.begin(PIN_EX1_DOUT, PIN_EX1_SCK);
  
  vTaskDelay(500);
  
  Serial.print("Tasks initalization...\n");
  RTOS_modeManager_queue = xQueueCreate(4, sizeof(uint8_t));
  RTOS_stepControl_queue = xQueueCreate(4, sizeof(uint32_t));
  RTOS_readings_queue = xQueueCreate(100, sizeof(struct reading));
  //RTOS_force_queue = xQueueCreate(100, sizeof(reading));
  //RTOS_ex1_queue = xQueueCreate(100, sizeof(reading));

  xTaskCreate(stepControl, "Step controller", 3000, NULL, 10, &RTOS_stepControl_handle);
  //xTaskCreate(limitCheck, "ISR limitCheck", 1000, NULL, 9, &RTOS_limitCheck_handle);
  xTaskCreate(dataReader, "Data sampling", 1000, NULL, 5, &RTOS_dataReader_handle);
  xTaskCreate(serialComm, "Serial communication", 2000, NULL, 4, &RTOS_serialComm_handle);
  xTaskCreate(modeManager, "Mode manager", 3000, NULL, 3, &RTOS_modeManager_handle);

  // wait for reset button to be released
  while(!digitalRead(PIN_ALT))
  {
    vTaskDelay(100);
  }

  attachInterrupt(PIN_ALT, emergency_trigger, FALLING);

  //attachInterrupt(PIN_ENDSTOP_1, endstop_trigger, CHANGE);
  //attachInterrupt(PIN_ENDSTOP_2, endstop_trigger, CHANGE);
  //attachInterrupt(PIN_ENDSTOP_3, endstop_trigger, CHANGE);
  //attachInterrupt(PIN_ENDSTOP_4, endstop_trigger, CHANGE);

  vTaskDelay(500);
  
  // notify setup ended
  Serial.println("All done! Hello.\n");
  ledcmd(LED_OK);
}

// loop has idle task hook, it runs when nothing else is going on
// left empty to allow resource cleanup
void loop() {}

void stepControl(void * parameter)
{
  uint32_t steps;
  bool dir; // true up, false down
  uint8_t step_active; // 0 both, 1 A, 2 B
  uint32_t step_delay;
  uint16_t step_delay_millis;
  uint16_t step_delay_micros;
  for (;;)
  {
    // we are using a queue to allow for different moving modes in future
    // also because multiple notification to task are NOT working (don't know why)
    xQueueReceive(RTOS_stepControl_queue, &steps, portMAX_DELAY);
    // notifications are used to stop
    ulTaskNotifyTake(pdTRUE, 0);  // clear previous stop notifications
    if (!serial_matlab) Serial.println("Stepper go command...");

    // this function can run for a very long time
    // unsubscribe TWDT from idle task to prevent crash
    //RTOS_idleTask_handle = xTaskGetIdleTaskHandle();
    //esp_task_wdt_delete(RTOS_idleTask_handle);

    // get movement variables
    dir = mainframe.getDir();
    step_active = mainframe.getSteppers();
    step_delay = mainframe.getDelay();
    step_delay_millis = floor(step_delay / 1000);
    step_delay_micros = step_delay % 1000;
    if (!serial_matlab) Serial.println((String)"Delay: "+step_delay+" millis: "+step_delay_millis+" micros: "+step_delay_micros);

    // set steppers direction and check if direction is free to move
    bool direction_allowed;
    if (dir)  // dir true is up
    {
      digitalWrite(PIN_DIR, HIGH);
      if (!serial_matlab) Serial.println("ALTO");
      direction_allowed = mainframe.cgUp();  // free space is up
    }
    else
    {
      digitalWrite(PIN_DIR, LOW);
      if (!serial_matlab) Serial.println("BASSO");
      direction_allowed = mainframe.cgDown();  // free space is down
    }

    // move correct steppers if there is space to do so
    // stopping will be handled by interrupt, we just want to now if we can start moving
    if (direction_allowed)
    {
      // wake up correct stepper driver
      switch (step_active)
      {
        case 0: // both steppers active
          {
            digitalWrite(PIN_STEPA, LOW);
            digitalWrite(PIN_STEPB, LOW);
          }
          break;
        case 1: // A stepper active
          {
            digitalWrite(PIN_STEPA, LOW);
            digitalWrite(PIN_STEPB, HIGH);
          }
          break;
        case 2: // B stepper active
          {
            digitalWrite(PIN_STEPA, HIGH);
            digitalWrite(PIN_STEPB, LOW);
          }
          break;
      }
      vTaskDelay(5);  // at least 1ms delay after waking up DRV8825 is needed

      if (!serial_matlab) Serial.println("Start moving...");

      // start moving based on amount requested
      switch (steps)
      {
        case MOVE_INDEF:
          {
            // ulTaskNotifyTake(pdTRUE, 0) returns notification value if there are notifications
            // ulTaskNotifyTake(pdTRUE, 0) returns 0 if there are no notifications
            // so !ulTaskNotifyTake(pdTRUE, 0) returns 1 if there are no notifications
            uint16_t step_delay_new;
            while (!ulTaskNotifyTake(pdTRUE, 0))  // do not wait if there is no notification
            {
              // we can move

              // this allows for changing speed without releasing movement button
              step_delay_new = mainframe.getDelay();
              if (step_delay_new != step_delay)
              {
                step_delay = step_delay_new;
                // calculate delay by using as little block time as possible with delayMicros
                // step_delay is > 1000 for sure so vTD always executes for at least 1ms
                step_delay_millis = floor(step_delay / 1000);
                step_delay_micros = step_delay % 1000;
                if (!serial_matlab) Serial.println((String)"Delay: "+step_delay+" millis: "+step_delay_millis+" micros: "+step_delay_micros);
              }

              step();

              delayMicroseconds(step_delay_micros);
              vTaskDelay(step_delay_millis);
            }
          }
          break;
        default:  // case is not indef, move for predefined space
          {
            // speed can not be changed in this mode
            while (!ulTaskNotifyTake(pdTRUE, 0) && steps > 0)
            {
              if (!serial_matlab) Serial.println((String)"Steps to go: "+steps);

              step();

              delayMicroseconds(step_delay_micros);
              vTaskDelay(step_delay_millis);
              steps--;
            }
          }
      }
      if (!serial_matlab) Serial.println("Stop");
    }
    // always disable all drivers
    digitalWrite(PIN_STEPA, HIGH);
    digitalWrite(PIN_STEPB, HIGH);
    // notify modeManager we have stopped
    xTaskNotifyGive(RTOS_modeManager_handle);
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
        // this is not needed because we can only get here with mainframe.setMode, mode is for sure updated
        //mainframe.setMode(mode); // set mainframe mode
        switch (mode)
        {
          // test modes
          case MODE_WEIGHT:
            {
              teststart();
              vTaskDelay(5000);
              testend();
            }
            break;
          case MODE_LENGTH:
            {
              teststart();
              if (!serial_matlab)
              {
                Serial.println((String)"--- INIZIO TEST ---");
                Serial.println((String)"Tempo: "+(millis() - test_millis_start));
                Serial.println((String)"Step richiesti: "+(move_target_microm * steps_per_microm));
              }
              mainframe.up(move_target_microm);
              ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // wait for movement stop
              testend();
            }
            break;
            // utility modes
          case MODE_CAL_UP:
            {
              ledcmd(LED_AUTO_UP);
              mainframe.setSpeed(50);
              mainframe.setSteppers(0);
              mainframe.setDelay(DELAY_MIN);  // max speed
              mainframe.up();
              ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // wait for movement stop
              // while (mainframe.cgUp())
              // {
              //   vTaskDelay(10); // check every 10ms if limit is reached
              // }

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
              ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // wait for movement stop
              // while (mainframe.cgUp())
              // {
              //   vTaskDelay(10);
              // }

              // now both sides have been pressed
              // slow down, reverse and level slowly
              mainframe.setSpeed(10);
              mainframe.setSteppers(1);
              mainframe.down();
              while (!mainframe.cgUp())
              {
                vTaskDelay(1);
              }
              mainframe.stop(); // stop when switch is released
              ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // wait for movement stop
              // now other stepper
              mainframe.setSteppers(2);
              mainframe.down();
              while (!mainframe.cgUp())
              {
                vTaskDelay(1);
              }
              mainframe.stop(); // stop when switch is released
              ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // wait for movement stop
              ledcmd(LED_OK);
            }
            break;
          case MODE_CAL_DOWN:
            {
              ledcmd(LED_AUTO_DOWN);
              mainframe.setSpeed(50);
              mainframe.setSteppers(0);
              mainframe.setDelay(DELAY_MIN);  // max speed
              mainframe.down();
              ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // wait for movement stop
              // while (mainframe.cgDown())
              // {
              //   vTaskDelay(10); // check every 10ms if limit is reached
              // }

              // now lower limit is reached, an endstop is pressed
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
              ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // wait for movement stop
              // while (mainframe.cgDown())
              // {
              //   vTaskDelay(10);
              // }

              // now both sides have been pressed
              // slow down, reverse and level slowly
              mainframe.setSpeed(50);
              mainframe.setSteppers(1);
              mainframe.up();
              while (!mainframe.cgDown())
              {
                vTaskDelay(1);
              }
              mainframe.stop(); // stop when switch is released
              ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // wait for movement stop
              // now other stepper
              mainframe.setSteppers(2);
              mainframe.up();
              while (!mainframe.cgDown())
              {
                vTaskDelay(1);
              }
              mainframe.stop(); // stop when switch is released
              ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // wait for movement stop
              ledcmd(LED_OK);
            }
            break;
          //default:  // default means we are either in manual or not recognized mode, exit from switch and enter manual mode
        }
        mainframe.setMode(MODE_MANUAL); // always set manual mode after executing automatic mode test
      }
    }
    else  // pdFALSE, there is no mode in queue so we are in manual mode
    {
      // MODE_MANUAL
      // read buttons
      btn_up.read();
      btn_down.read();

      // calculate speed
      if (speed_read_encoder)
      {
        uint16_t speed_raw = analogRead(PIN_SPEED);
        uint8_t encoder_speed_new = map(speed_raw, 0, 4095, 1, SPEED_STEPS);
        if(encoder_speed_new != encoder_speed)
        {
          encoder_speed = encoder_speed_new;
          mainframe.setSpeedInt(encoder_speed);
        }
      }
      
      // select movement
      if (btn_up.isPressed() && btn_down.isPressed()) // should not press both, failsafe stops and starts calibration cooldown
      {
        if (btn_up.justPressed() || btn_down.justPressed()) // at least one button has just been pressed
        {
          mainframe.stop();
          ledcmd(LED_12);
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
            ledcmd(LED_AUTO_SELECT);
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
        if (btn_up.justPressed() && mainframe.cgUp()) // check movement bounds
        {
          mainframe.stop();
          mainframe.up();
          ledcmd(LED_1);
        }
        else if (btn_down.justPressed() && mainframe.cgDown())
        {
          mainframe.stop();
          mainframe.down();
          ledcmd(LED_2);
        }
        else if (btn_up.justReleased() || btn_down.justReleased())
        {
          //Serial.println(millis());
          mainframe.stop();
          //Serial.println(millis());
          ledcmd(LED_OFF);
        }  
      }

      // check if calibration must start
      vTaskDelay(MANUAL_FREQ); // check every MANUAL_FREQ ms
    }
  }
}

void dataReader(void * parameter)
{
  TickType_t xLastWakeTime = xTaskGetTickCount();
  TickType_t xFrequency = 1000/loadcell_hz;
  //float data_raw;

  // struct reading
  // {
  //   uint8_t mode;
  //   uint16_t speed; // in micron/min
  //   uint32_t timestamp; // millis
  //   uint16_t force; // in 0.1N
  //   uint16_t ex1; // in micron
  // }

  struct reading reading_current;
  for (;;)
  {
    xFrequency = 1000/loadcell_hz;
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    reading_current.mode = mainframe.getMode();
    reading_current.speed = round(mainframe.getSpeed() * 100);
    reading_current.timestamp = millis() - test_millis_start;
    //reading_last.force = round(loadcell.get_units(5) * 10);
    reading_current.force = 111;
    reading_current.ex1 = test_steps / steps_per_microm;

    reading_last = reading_current; // save changes to global variable
    xQueueSend(RTOS_readings_queue, &reading_current, 100);
    //Serial.println("Sent reading...");
  }
}

void serialComm(void * parameter)
{
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = 100; // delay for mS

  struct reading reading_send;

  for(;;)
  {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    // receive data
    while (Serial.available() > 0)
    {
      if (serial_matlab)  // matlab communication protocol
      {
        int8_t inByte = Serial.read();
        if (inByte == CMD_MATLAB_START)
        {
          uint8_t cmd[CMD_MATLAB_LENGTH];
          Serial.readBytes(cmd, CMD_MATLAB_LENGTH);
          // send ack to matlab
          Serial.write(CMD_MATLAB_ACK);
          // byte 1,2 are speed
          // speed is passed in 0.01mm/min = 10 microm/min
          // divide by 100 to get mm/min
          uint16_t speed_new = (cmd[1] * 256 + cmd[2]);
          speed_read_encoder = false; // disable speed encoder
          mainframe.setSpeed(speed_new);
          // byte 3,4 are force
          force_target = cmd[3] * 256 + cmd[4];
          // byte 5,6 are extension
          // extension is passed in microm
          ex1_target = cmd[5] * 256 + cmd[6];
          // we can use extension but it is only relative
          // we use target in steps because it is the only thing we can measure
          // BEWARE we have no way to know is there are skipped steps, it is only a target
          move_target_microm = ex1_target;
          ledcmd(LED_OK);
          // send ack again to matlab after a while
          vTaskDelay(10);
          Serial.write(CMD_MATLAB_ACK);
          // clear buffer
          while (Serial.available() > 0)
          {
            inByte = Serial.read();
          }
          // byte 0 is mode
          // it is set last to prevent from starting test early
          // without all the config data provided in the message
          if (cmd[0] == 0)
          {
            ESP.restart();
          }
          else
          {
            mainframe.setMode(cmd[0]);
          }
          // else
          // {
          //   // return wrong command
          //   Serial.println("Wrong command length!");
          // }
        }
        else if (inByte == SERIAL_MANUAL) // this is used to switch from MATLAB to serial via arduino serial monitor
        {
          ledcmd(LED_OK);
          serial_matlab = false;
          Serial.println("Switching to manual command mode...");
          // clear buffer
          while (Serial.available() > 0)
          {
            inByte = Serial.read();
          }
        }
        // message was invalid, start searching again for start byte
        // we do nothing because serial available restart whole loop


        // --- DEPRECATED ---
        // message was invalid, remove everyhting from buffer
        // we remove everything because we aren't sure when next good message starts
        // else
        // {
        //   //Serial.println((String)inByte+" is not a MATLAB start code");
        //   // clear buffer
        //   while (Serial.available() > 0)
        //   {
        //     inByte = Serial.read();
        //   }
        //  }       
      }
      else  // serial monitor communication protocol
      {   
        // create a place to hold the incoming command
        char cmd[CMD_SERIAL_LENGTH];
        uint8_t cmd_pos = 0;
        float cmd_int;
        // read the next available byte in the serial receive buffer
        char inByte = Serial.read();
        
        // command coming in (check not terminating character) and guard for over command size
        while (inByte != '\n' && (cmd_pos < CMD_SERIAL_LENGTH - 1))
        {
          // add the incoming byte to our command
          cmd[cmd_pos] = inByte;
          cmd_pos++;
          inByte = Serial.read();
        }
        // full cmd received
        // add null character to string
        cmd[cmd_pos] = '\0';

        Serial.println((String)"Command received: "+cmd);
        // we do not make any input validation
        // please make sure incoming data is less than 16 character long
        // and is also a single command formatted like M1234 (16 char max total)
        switch (cmd[0]) // first char is data type
        {
          case 'X': // switch to MATLAB interface
            {
              serial_matlab = true;
              Serial.println("\n\nSwitching to MATLAB command mode...");
              ledcmd(LED_OK);
            }
            break;
          case 'M': // mode has been sent
            {
              cmd_int = cmdtoi(cmd);
              if (cmd_int == MODE_MANUAL)
              {
                // this means we have called and abort command
                // restarting ESP is the only way to make sure we terminate every task
                Serial.println("Rebooting, see you soon!");
                ESP.restart();
              }
              else // any other mode is the beginning of a test
              {
                mainframe.setMode(cmd_int);
              }
              ledcmd(LED_OK);
            }
            break;
          case 'S': // stepper to move has been sent
            {
              cmd_int = cmdtoi(cmd);
              // input validation
              if (cmd_int < 0 || cmd_int > 2)
              {
                cmd_int = 0;
              }
              mainframe.setSteppers(cmd_int);
              ledcmd(LED_OK);
            }
            break;
          case 'D': // delay setting has been sent
            {
              cmd_int = cmdtoi(cmd);
              // delay 0 enables the encoder speed reader
              if (cmd_int == 0)
              {
                speed_read_encoder = true;
              }
              else
              {
                speed_read_encoder = false;
                mainframe.setDelay(cmd_int);
              }
              ledcmd(LED_OK);
            }
            break;
          case 'V': // speed has been sent
            {
              cmd_int = cmdtoi(cmd);
              // speed 0 enables the encoder speed reader
              if (cmd_int == 0)
              {
                speed_read_encoder = true;
              }
              else
              {
                speed_read_encoder = false;
                mainframe.setSpeed(cmd_int);
              }
              ledcmd(LED_OK);
            }
            break;
          case 'J': // space to jiggle has been sent in microm
            {
              cmd_int = cmdtoi(cmd);
              if (cmd_int > 0)
              {
                mainframe.up(cmd_int);
              }
              else if (cmd_int < 0)
              {
                mainframe.down(-cmd_int); // we need to send positive value
              }
              else  // jiggle 0 stops the steppers
              {
                mainframe.stop();
              }
              ledcmd(LED_OK);
            }
            break;
          default:
            Serial.println("Unknown initalizer!");
        }
        Serial.println((String)"Command value: "+cmd_int);
        // reset for the next command
        cmd_pos = 0;
      }
    }
    // transimt data
    while (xQueueReceive(RTOS_readings_queue, &reading_send, 0) == pdTRUE)
    {
      if (serial_matlab)
      {
        // parse message from reading
        uint8_t outByte[CMD_MATLAB_OUTPUT];
        outByte[0] = CMD_MATLAB_START;
        outByte[1] = (reading_send.mode >> 0) & 0xFF;;
        uint16_t speed_raw = reading_send.speed;
        outByte[2] = (speed_raw >> 8) & 0xFF;
        outByte[3] = (speed_raw >> 0) & 0xFF;
        // outByte[2] = (int8_t) speed_raw>>8;
        // outByte[3] = (int8_t) speed_raw % pow(2, 8);
        uint32_t time_relative = reading_send.timestamp;
        outByte[4] = (time_relative >> 24) & 0xFF;
        outByte[5] = (time_relative >> 16) & 0xFF;
        outByte[6] = (time_relative >> 8) & 0xFF;
        outByte[7] = (time_relative >> 0) & 0xFF;
        uint32_t load_raw = reading_send.force;
        outByte[8] = (load_raw >> 8) & 0xFF;
        outByte[9] = (load_raw >> 0) & 0xFF;
        uint32_t ex1_raw = reading_send.ex1;
        outByte[10] = (ex1_raw >> 8) & 0xFF;
        outByte[11] = (ex1_raw >> 0) & 0xFF;

        // send message
        vTaskPrioritySet(RTOS_serialComm_handle, 9);  // prevent task from interrupting while sending
        Serial.write(outByte, CMD_MATLAB_OUTPUT);
        vTaskPrioritySet(RTOS_serialComm_handle, 4);  // restore default priority
      }
      else
      {
        // Serial.println("--- VALORI CORRENTI ---");
        // Serial.println((String)"Speed: "+reading_send.speed/100);
        // Serial.println((String)"Timestamp: "+reading_send.timestamp);
        // Serial.println((String)"Force: "+reading_send.force);
        // Serial.println((String)"Extension: "+reading_send.ex1);
      }
    }
  }
}

void ledManager(void * parameter)
{
  uint8_t cmd;
  for (;;)
  {
    xQueueReceive(RTOS_ledManager_queue, &cmd, portMAX_DELAY);
    ulTaskNotifyTake(pdTRUE, 0);  // clear previous notifications
    //Serial.println((String)"Led: "+cmd);

    switch (cmd)
    {
      case LED_OFF:
        {
          led1.off();
          led2.off();
          led3.off();
        }
        break;
      case LED_ERROR:
        {
          led3.on();
          vTaskDelay(500);
          ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // keep led on if code has an issue and doesn't unlock
          led1.on();
          vTaskDelay(500);
          led2.on();
          vTaskDelay(500);
          led1.off();
          led2.off();
          led3.off();
        }
        break;
      case LED_OK:
        {
          for (uint8_t i = 0; i < 3; ++i)
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
          ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
          led1.off();
        }
        break;
      case LED_2:
        {
          led2.on();
          ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
          led2.off();
        }
        break;
      case LED_12:
        {
          led1.on();
          led2.on();
          ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
          led1.off();
          led2.off();
        }
        break;
      case LED_3:
        {
          led3.on();
          ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
          led3.off();
        }
        break;
      case LED_AUTO_SELECT:
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
        }
        break;
      case LED_AUTO_UP:
        {
          led1.on();
          led3.on();
          ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
          led1.off();
          led3.off();
        }
        break;
      case LED_AUTO_DOWN:
        {
          led2.on();
          led3.on();
          ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
          led2.off();
          led3.off();
        }
        break;
    }
    vTaskDelay(50); // min 50ms between messages
  }
}
