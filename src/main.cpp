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
  options.toolRadius = 0.005f;
  options.cornerTreatment = CORNER_ROLL;
  options.globalTrimCrossing = true;

  const int lines = (int)(sizeof(demo_program) / sizeof(demo_program[0]));
  bool ok = runner.begin(options);

  unsigned long start = millis();
  for (int i = 0; ok && i < lines; ++i)
    ok = runner.processLine(demo_program[i]);
  if (ok)
    ok = runner.finish();
  unsigned long end = millis();

  Serial.print("Elapsed time (us): ");
  Serial.println(end - start);

  if (!ok)
    Serial.println("(run failed)");

}

void loop()
{
}
