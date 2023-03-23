#include "classes.h"


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

void teststart()
{
  ledcmd(LED_3);
  loadcell_hz = 10;
  vTaskDelay(2000);
  ledcmd(LED_AUTO_UP);
  test_millis_start = millis();
  test_steps = 0;
}

void testend()
{
  struct reading reading_end;

  // we can send a fake mode to MATLAB so it stops the test, even if mainframe is in another mode
  reading_end.mode = MODE_TESTEND;
  reading_end.speed = reading_last.speed;
  reading_end.timestamp = millis() - test_millis_start;
  reading_end.force = reading_last.force;
  reading_end.ex1 = test_steps / steps_per_microm;

  xQueueSend(RTOS_readings_queue, &reading_end, portMAX_DELAY);
  ledcmd(LED_3);

  if (!serial_matlab)
  {
    Serial.println((String)"--- RISULTATI TEST ---");
    Serial.println((String)"Tempo: "+reading_end.timestamp);
    Serial.println((String)"Step finali: "+test_steps);
    Serial.println((String)"Forza finale: "+reading_end.force);
    Serial.println((String)"Estensione finale: "+reading_end.ex1);
  }

  vTaskDelay(2000);
  ledcmd(LED_OK);
  loadcell_hz = 1;
  speed_read_encoder = true;  // enable speed encoder
}

void ledcmd(uint8_t code)
{
  xTaskNotifyGive(RTOS_ledManager_handle);  // clears any previous cmd
  xQueueSend(RTOS_ledManager_queue, &code, 0);
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

