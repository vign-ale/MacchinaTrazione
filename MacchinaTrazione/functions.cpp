#include "definitions.h"

void step()
{
  digitalWrite(PIN_STEP, HIGH);
  delayMicroseconds(pulse_length);
  digitalWrite(PIN_STEP, LOW);
  test_steps ++;
}

void teststart(float frequency)
{
  ledcmd(LED_3);
  loadcell_hz = frequency;
  vTaskDelay(1000);
  ledcmd(LED_AUTO_UP);
  test_millis_start = millis();
  test_steps = 0;
  //if (!serial_matlab) Serial.println((String)"\n--- OK ---");
  //else Serial.println((String)"\n--- ERR ---");
  if (!serial_matlab)
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
}
void teststart()
{
  teststart(10);  // default test frequency
}

void testend()
{
  struct reading reading_end;

  // we can send a fake mode to MATLAB so it stops the test, even if mainframe is in another mode
  reading_end.mode = MODE_TESTEND;
  reading_end.speed = reading_last.speed;
  reading_end.timestamp = millis() - test_millis_start;
  reading_end.force = reading_last.force;
  reading_end.position = test_steps / steps_per_microm;

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

  vTaskDelay(2000);
  ledcmd(LED_OK);
  loadcell_hz = 1;
  speed_read_encoder = true;  // enable speed encoder
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
  loadcell.set_scale(value);
  memory.begin("loadcell", false);
  memory.putULong("calibration", value);
  memory.end();
  if (!serial_matlab) Serial.println((String)"Set loadcell calibration factor: "+value);
}
