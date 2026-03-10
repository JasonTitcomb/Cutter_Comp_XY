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
#ifdef ARDUINO
  Serial.write((const uint8_t *)text, len);
#else
  std::fwrite(text, 1, len, stdout);
#endif
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
#ifdef ARDUINO
  while (!Serial)
  {
  }
#endif

  CcMainRunner runner;
  CcMainOptions options;
  CcMainCallbacks callbacks;
  callbacks.output = serial_output_cb;
  callbacks.error = serial_error_cb;

  const int lines = (int)(sizeof(demo_program) / sizeof(demo_program[0]));
  bool ok = runner.begin(options, callbacks);

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
