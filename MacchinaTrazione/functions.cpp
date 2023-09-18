#include "definitions.h"

void step(bool up)
{
  digitalWrite(PIN_STEP, HIGH);
  delayMicroseconds(pulse_length);
  digitalWrite(PIN_STEP, LOW);
  if (up)
  {
    test_steps ++;
  }
  else if (test_steps > 0)  // do not allow negative postion
  {
    test_steps --;
  }
  
}

void teststart(float frequency)
{
  ledcmd(LED_3);
  test_steps = 0;
  test_millis_start = millis();
  loadcell_hz = frequency;
  if (serial_matlab)
  {
    struct reading reading_start;
    // send first datapoint
    reading_start.mode = mainframe.getMode();
    reading_start.speed = round(mainframe.getSpeed() * 100);
    reading_start.timestamp = millis() - test_millis_start;
    reading_start.force = round(abs(loadcell.get_units(LOADCELL_READINGS)) * 10);
    reading_start.position = round(test_steps / steps_per_microm);

    xQueueSend(RTOS_readings_queue, &reading_start, portMAX_DELAY);
  }
  else
  {
    Serial.println((String)"\n--- INIZIO TEST ---");
    Serial.println((String)"Tempo: "+(millis() - test_millis_start));
    uint8_t test_type = mainframe.getMode();
    switch (test_type)
    {
      case MODE_WEIGHT:
      {
        Serial.println((String)"Forza richiesta: "+force_target+"\n");
      }
      break;
      case MODE_LENGTH:
      {
        Serial.println((String)"Step richiesti: "+(position_target * steps_per_microm)+"\n");
      }
      break;
      default:
      {
        Serial.println((String)"Errore modalità test!\n");
      }
    }
  }
  vTaskDelay(1000); // wait for load value to stabilize before moving
  ledcmd(LED_AUTO_UP);
}
void teststart()
{
  teststart(5);  // default test frequency
}

void testend()
{
  struct reading reading_end;

  // we can send a fake mode to MATLAB so it stops the test, even if mainframe is in another mode
  reading_end.mode = MODE_TESTEND;
  reading_end.speed = round(mainframe.getSpeed() * 100);
  reading_end.timestamp = millis() - test_millis_start;
  reading_end.force = round(abs(loadcell.get_units(LOADCELL_READINGS)) * 10);
  reading_end.position = round(test_steps / steps_per_microm);

  xQueueSend(RTOS_readings_queue, &reading_end, portMAX_DELAY);
  ledcmd(LED_3);

  if (!serial_matlab)
  {
    Serial.println((String)"\n--- RISULTATI TEST ---");
    Serial.println((String)"Tempo: "+reading_end.timestamp);
    Serial.println((String)"Step finali: "+test_steps);
    Serial.println((String)"Forza finale: "+reading_end.force);
    Serial.println((String)"Estensione finale: "+reading_end.position+"\n");
  }

  vTaskDelay(1000);
  ledcmd(LED_OK);
  loadcell_hz = 1;
  speed_read_encoder = true;  // enable speed encoder
  mainframe.setSpeedInt(encoder_speed);
}

void ledcmd(uint8_t code)
{
  //if (!serial_matlab) Serial.println((String)"Led richiesto: "+code);
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

uint32_t loadcellGetCal()
{
  memory.begin("loadcell", true);
  uint32_t value = memory.getULong("calibration", 0);
  memory.end();
  if (!serial_matlab) Serial.println((String)"Read loadcell calibration factor: "+value);
  return value;
}

void loadcellSetCal(uint32_t value)
{
  if (value == 0)
  {
    loadcell.set_scale();
  }
  else
  {
    loadcell.set_scale(value);
  }
  memory.begin("loadcell", false);
  memory.putULong("calibration", value);
  memory.end();
  if (!serial_matlab) Serial.println((String)"Set loadcell calibration factor: "+value);
}
