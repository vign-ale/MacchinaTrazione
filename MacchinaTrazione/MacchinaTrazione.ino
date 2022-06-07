#include "classes.h"

//input btn_manual;
//input btn_confirm;
uint8_t encoder_speed = 1;
bool speed_read_encoder = true;

// input endstops[4] = { input(PIN_ENDSTOP_1, true), input(PIN_ENDSTOP_2, true), input(PIN_ENDSTOP_3, true), input(PIN_ENDSTOP_4, true)};
input endstops[4] = {{PIN_ENDSTOP_1, false}, {PIN_ENDSTOP_2, false}, {PIN_ENDSTOP_3, false}, {PIN_ENDSTOP_4, false}};
// input btn_confirm(PIN_CONFIRM, false);
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
float loadcell_hz = 0.2;

struct reading
{
  uint8_t mode;
  uint16_t speed; // in 0.01 mm/min
  uint32_t timestamp; // millis
  uint16_t force; // in 0.1N
  uint16_t ex1; // in TBD
};

float force_current = 0;  // value in N
float force_target = 0;
float ex1_current = 0;  // value in ?
float ex1_target = 0;
float move_target_microm = 1000;

uint16_t millis_elapsed;

// stepper configuration
uint8_t pulse_length = 15;
float steps_per_mm = 200 * 1 * 26.85 / 5; // Steps per rev * Microstepping * Gear reduction ratio / Pitch

frame mainframe;
uint32_t test_millis_start = 0;
uint32_t test_steps = 0;

// serial configuration
bool serial_matlab = true;


// RTOS stuff
TaskHandle_t RTOS_stepControl_handle;
QueueHandle_t RTOS_stepControl_queue;
// this semaphore taken means red: stop the movement
// if semaphore free movement can happen 
//SemaphoreHandle_t RTOS_stepControl_semaphore;
TaskHandle_t RTOS_modeManager_handle;
QueueHandle_t RTOS_modeManager_queue;
TaskHandle_t RTOS_ledManager_handle;
QueueHandle_t RTOS_ledManager_queue;
TaskHandle_t RTOS_limitCheck_handle;
TaskHandle_t RTOS_serialComm_handle;
TaskHandle_t RTOS_dataReader_handle;
QueueHandle_t RTOS_readings_queue;
//QueueHandle_t RTOS_force_queue;
//QueueHandle_t RTOS_ex1_queue;

// here we use an ISR to activate a task that runs checkLimit
// the task is not used elsewhere as it is better to make it run in other task
// this way we are always sure the checkLimit ended before anything else happens in the task
// in this case we come from an ISR so the task has really high priority and will be run ASAP
void IRAM_ATTR endstop_trigger()
{
  // use xHigherPriorityTaskWoken to make sure context switch is as fast as possible
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  vTaskNotifyGiveFromISR(RTOS_limitCheck_handle, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void limitCheck(void * parameter)
{
  for (;;)
  {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    mainframe.checkLimit();
  }
}


void setup()
{
  Serial.begin(115200);
  Serial.println("Booting...");
  RTOS_ledManager_queue = xQueueCreate(4, sizeof(uint8_t));
  xTaskCreate(ledManager, "Led manager", 3000, NULL, 2, &RTOS_ledManager_handle);
  ledcmd(LED_ERROR);
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

  RTOS_modeManager_queue = xQueueCreate(4, sizeof(uint8_t));
  RTOS_stepControl_queue = xQueueCreate(4, sizeof(uint32_t));
  RTOS_readings_queue = xQueueCreate(200, sizeof(struct reading));
  //RTOS_force_queue = xQueueCreate(100, sizeof(reading));
  //RTOS_ex1_queue = xQueueCreate(100, sizeof(reading));

  xTaskCreate(stepControl, "Step controller", 3000, NULL, 10, &RTOS_stepControl_handle);
  xTaskCreate(limitCheck, "ISR limitCheck", 1000, NULL, 9, &RTOS_limitCheck_handle);
  xTaskCreate(serialComm, "Serial communication", 2000, NULL, 8, &RTOS_serialComm_handle);
  xTaskCreate(dataReader, "Data sampling", 1000, NULL, 4, &RTOS_dataReader_handle);
  xTaskCreate(modeManager, "Mode manager", 3000, NULL, 3, &RTOS_modeManager_handle);

  Serial.print("Mainframe initalization...");
  mainframe.init();
  pinMode(PIN_STEPA, OUTPUT);
  pinMode(PIN_STEPB, OUTPUT);
  pinMode(PIN_DIR, OUTPUT);

  attachInterrupt(PIN_ENDSTOP_1, endstop_trigger, CHANGE);
  attachInterrupt(PIN_ENDSTOP_2, endstop_trigger, CHANGE);
  attachInterrupt(PIN_ENDSTOP_3, endstop_trigger, CHANGE);
  attachInterrupt(PIN_ENDSTOP_4, endstop_trigger, CHANGE);

  loadcell.begin(PIN_LOADCELL_DOUT, PIN_LOADCELL_SCK);
  loadcell.begin(PIN_EX1_DOUT, PIN_EX1_SCK);

  // notify setup ended
  Serial.println(" done!");
  Serial.println("All done! Hello.");
  ledcmd(LED_OK);
}


// loop has idle task hook, it runs when nothing else is going on
// left empty to allow resource cleanup
void loop() {}

void ledcmd(uint8_t code)
{
  xTaskNotifyGive(RTOS_ledManager_handle);  // clears any previous cmd
  xQueueSend(RTOS_ledManager_queue, &code, 0);
}

void stepControl(void * parameter)
{
  uint32_t move_amount;
  bool dir;
  uint8_t step_active; // 0 both, 1 A, 2 B
  uint32_t step_delay;
  uint16_t step_delay_millis;
  uint16_t step_delay_micros;
  for (;;)
  {
    // we are using a queue to allow for different moving modes in future
    // also because multiple notification to task are NOT working (don't know why)
    xQueueReceive(RTOS_stepControl_queue, &move_amount, portMAX_DELAY);
    ulTaskNotifyTake(pdTRUE, 0);  // clear previous stop notifications

    // this function can run for a very long time
    // unsubscribe TWDT from idle task to prevent crash
    //RTOS_idleTask_handle = xTaskGetIdleTaskHandle();
    //esp_task_wdt_delete(RTOS_idleTask_handle);

    // get movement variables
    dir = mainframe.getDir();
    step_active = mainframe.getSteppers();
    step_delay = mainframe.getDelay();
    step_delay_millis = step_delay / 1000;
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
            digitalWrite(PIN_STEPA, HIGH);
            digitalWrite(PIN_STEPB, HIGH);
          }
          break;
        case 1: // A stepper active
          {
            digitalWrite(PIN_STEPA, HIGH);
          }
          break;
        case 2: // B stepper active
          {
            digitalWrite(PIN_STEPB, HIGH);
          }
          break;
      }

      if (!serial_matlab) Serial.println("Start moving...");

      // start moving based on amount requested
      switch (move_amount)
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
                step_delay_millis = step_delay / 1000;
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
            // movement is defined in thousands of a mm
            uint32_t steps = round(move_amount * steps_per_mm / 1000);
            while (!ulTaskNotifyTake(pdTRUE, 0) && steps > 0)
            {
              //if (!serial_matlab) Serial.println((String)"Steps to go: "+steps);
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
    digitalWrite(PIN_STEPA, LOW);
    digitalWrite(PIN_STEPB, LOW);
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
        mainframe.setMode(mode); // set mainframe mode
        test_millis_start = millis();
        test_steps = 0;
        switch (mode)
        {
          // test modes
          case MODE_WEIGHT:
            {
              vTaskDelay(5000);
            }
            break;
          case MODE_LENGTH:
            {
              
              ledcmd(LED_AUTO_UP);
              uint32_t move_target_step = move_target_microm * steps_per_mm / 1000;
              if (!serial_matlab)
              {
                Serial.println((String)"--- INIZIO TEST TEST ---");
                Serial.println((String)"Tempo: "+(millis() - test_millis_start));
                Serial.println((String)"Step richiesti: "+move_target_step);
                Serial.println((String)"Step iniziali: "+test_steps);
              }
              mainframe.up(move_target_step);
              ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // wait for movement stop
              if (!serial_matlab)
              {
                Serial.println((String)"--- RISULTATI TEST ---");
                Serial.println((String)"Tempo: "+(millis() - test_millis_start));
                Serial.println((String)"Step richiesti: "+move_target_step);
                Serial.println((String)"Step finali: "+test_steps);
              }
              ledcmd(LED_OK);
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
              mainframe.setSpeed(5);
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
              mainframe.setSpeed(5);
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
      // read buttons
      btn_up.read();
      btn_down.read();

      // calculate speed
      if (speed_read_encoder)
      {
        uint16_t speed_raw = analogRead(PIN_SPEED);
        encoder_speed = map(speed_raw, 0, 4095, 1, SPEED_STEPS);
        mainframe.setSpeedInt(encoder_speed);
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
          led2.on();
          vTaskDelay(500);
          led1.on();
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

void dataReader(void * parameter)
{
  TickType_t xLastWakeTime = xTaskGetTickCount();
  TickType_t xFrequency = 1000/loadcell_hz;
  //float data_raw;

  // struct reading
  // {
  //   uint8_t mode;
  //   uint16_t speed; // in 0.01 mm/min
  //   uint32_t timestamp; // millis
  //   uint16_t force; // in 0.1N
  //   uint16_t ex1; // in TBD
  // }

  struct reading reading_last;
  for (;;)
  {

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    reading_last.mode = mainframe.getMode();
    reading_last.speed = round(mainframe.getSpeed() * 100);
    reading_last.timestamp = millis() - test_millis_start;
    //reading_last.force = round(loadcell.get_units(5) * 10);
    reading_last.force = 111;
    reading_last.ex1 = 999;

    //p_reading_last = & reading_last;
    xQueueSend(RTOS_readings_queue, &reading_last, 100);
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
          //Serial.println("Recognized...");
          uint8_t cmd[CMD_MATLAB_LENGTH];

          // DEPRECATED: we trust the message composition
          // serial.read() returns -1 if no data is available (wrong message)
          // we use this to exclude bad messages
          // bool error = false;
          // for (uint8_t i = 0; i < CMD_MATLAB_LENGTH; ++i)
          // {
          //   inByte = Serial.read();
          //   Serial.println(inByte);
          //   if (inByte == -1)
          //   {
          //     i = CMD_MATLAB_LENGTH;
          //     error = true;
          //   }
          //   else
          //   {
          //     cmd[i] = inByte;
          //   }
          // }
          ;
          // if (!error) // so all message was ok

          Serial.readBytes(cmd, CMD_MATLAB_LENGTH);
          // byte 1,2 are speed
          // speed is passed in 0.01mm/min = 10 microm/min
          // divide by 100 to get mm/min
          float speed_new = (cmd[1] * 256 + cmd[2]) / 100;
          mainframe.setSpeed(speed_new);
          // byte 3,4 are extension
          // extension is passed in microm
          ex1_target = cmd[3] * 256 + cmd[4];
          // we can use extension but it is only relative
          // we use target in steps because it is the only thing we can measure
          // BEWARE we have no way to know is there are skipped steps, it is only a target
          move_target_microm = ex1_target;
          // byte 5,6 are force
          force_target = cmd[5] * 256 + cmd[6];;
          // send ack to matlab
          Serial.write(CMD_MATLAB_ACK);
          ledcmd(LED_OK);
          // byte 0 is mode
          // it is set last to prevent from starting test early
          // without all the config data provided in the message
          vTaskDelay(2000); // wait a while before starting
          mainframe.setMode(cmd[0]);
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
              Serial.println("\nSwitching to MATLAB command mode...");
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
          case 'J': // space to jiggle has been sent
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
    //Serial.println("Received reading!");
      if (serial_matlab)
      {
        // parse message from reading
        int8_t outByte[CMD_MATLAB_OUTPUT];
        outByte[0] = CMD_MATLAB_START;
        outByte[1] = (int8_t) reading_send.mode;
        uint32_t speed_raw = reading_send.speed;
        outByte[2] = (int8_t) floor(speed_raw / pow(2, 8));
        outByte[3] = (int8_t) speed_raw - outByte[2] * pow(2, 8);
        uint32_t time_relative = reading_send.timestamp - test_millis_start;  // subtract test start time
        outByte[4] = (int8_t) floor(time_relative / pow(2, 24));
        time_relative = time_relative - outByte[4] * pow(2, 24);
        outByte[5] = (int8_t) floor(time_relative / pow(2, 16));
        time_relative = time_relative - outByte[5] * pow(2, 16);
        outByte[6] = (int8_t) floor(time_relative / pow(2, 8));
        time_relative = time_relative - outByte[6] * pow(2, 8);
        outByte[7] = (int8_t) floor(time_relative);
        uint32_t load_raw = reading_send.speed;
        outByte[8] = (int8_t) floor(load_raw / pow(2, 8));
        outByte[9] = (int8_t) load_raw - outByte[2] * pow(2, 8);
        uint32_t ex1_raw = reading_send.speed;
        outByte[10] = (int8_t) floor(ex1_raw / pow(2, 8));
        outByte[11] = (int8_t) ex1_raw - outByte[2] * pow(2, 8);

        // send message
        for (uint8_t i = 0; i < CMD_MATLAB_OUTPUT; ++i)
        {
          Serial.write(outByte[i]);
        }
      }
      else
      {
        Serial.println("--- VALORI CORRENTI ---");
        Serial.println((String)"Speed: "+reading_send.speed/100);
        Serial.println((String)"Timestamp: "+reading_send.timestamp);
        Serial.println((String)"Force: "+reading_send.force);
        Serial.println((String)"Extension: "+reading_send.ex1);
      }
    }
  }
}

float cmdtoi(char *cmd)
{
  // dump the character wich only tells us the type
  for (uint8_t i = 0; i < CMD_SERIAL_LENGTH - 1; ++i)
  {
    cmd[i] = cmd[i+1];
  }
  float cmd_int = atof(cmd);
  return cmd_int;
}

void step()
{
  digitalWrite(PIN_STEP, HIGH);
  delayMicroseconds(pulse_length);
  digitalWrite(PIN_STEP, LOW);
  test_steps ++;
  // switch (stepper)
  // {
  //   case 0: // both steppers active
  //   { 
  //     // NOTE: digitalWrite funtion is not fast to execute, but the delay should be << compared to pulse_lenght
  //     // this means there is a slight asymmetry and delay between the right and left steppers, but it should not be an issue
  //     // TODO: check if it's really an issue or not
  //     digitalWrite(PIN_STEPA, HIGH);
  //     digitalWrite(PIN_STEPB, HIGH);
  //     delayMicroseconds(pulse_length);
  //     digitalWrite(PIN_STEPA, LOW);
  //     digitalWrite(PIN_STEPB, LOW);
  //   }
  //   case 1: // stepper A active
  //   {
  //     digitalWrite(PIN_STEPA, HIGH);
  //     delayMicroseconds(pulse_length);
  //     digitalWrite(PIN_STEPA, LOW);
  //   }
  //   case 2: // stepper B active
  //   {
  //     digitalWrite(PIN_STEPB, HIGH);
  //     delayMicroseconds(pulse_length);
  //     digitalWrite(PIN_STEPB, LOW);
  //   }
  // }
}



