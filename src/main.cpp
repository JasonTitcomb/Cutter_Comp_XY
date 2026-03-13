#ifdef ARDUINO
#include <Arduino.h>
#else
#include "ArduinoCompat.h"
#include <cstdio>
#endif

#include "cc_main.h"
#include "test_data.h"

static constexpr uint32_t BAUD = 115200;


static void serial_output_cb(const char *text, size_t len)
{
  Serial.write((const uint8_t *)text, len);

}

static void serial_error_cb(const char *message, CompError err, uint32_t seqNum)
{
  (void)err;
  (void)seqNum;

  Serial.println(message);

}

static void start_comp_cb(int toolRegister, int diaRegister)
{
  Serial.print("Start comp with tool register ");
  Serial.print(toolRegister);
  Serial.print(" and dia register ");
  Serial.println(diaRegister);
}

void setup()
{
  Serial.begin(BAUD);
 while (!Serial)
  {
  }

  CcMainRunner runner;
  CcMainOptions options;
  options.callbacks.output = serial_output_cb;
  options.callbacks.error = serial_error_cb;
  options.callbacks.startComp = start_comp_cb;
  options.toolRadius = 0.005f;
  options.cornerTreatment = CORNER_ROLL;
  options.globalTrimCrossing = true;
  options.outputInchUnits = true;
  options.absoluteMode = false;
 

  const int lines = (int)(sizeof(demo_program) / sizeof(demo_program[0]));
  bool ok = runner.begin(options);

  for (int i = 0; ok && i < lines; ++i)
    ok = runner.processLine(demo_program[i]);

  if (ok)
    ok = runner.finish();

  if (!ok)
    Serial.println("(run failed)");

}

void loop()
{
}
