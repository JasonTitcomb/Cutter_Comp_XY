#include <Arduino.h>

#include "TinyGCodeScan.h"
#include "CutterComp2D.h"
#include "TestData.h"

// -------------------- Config --------------------
static constexpr uint32_t BAUD = 115200;

// IMPORTANT: Tool radius must match the units of your G-code.
// Your demo program looks like inches. Example 1/8" endmill radius:
static constexpr float TOOL_RADIUS = 0.0625f;

// Toggle compensation on/off for A/B testing
static constexpr bool ENABLE_COMP   = true;
static constexpr bool ENABLE_FILLET = true;

// ------------------------------------------------

static ModalState modal;
static CutterComp2D cc;


static void emit_move_as_gcode(const Move2D& m)
{
  if (m.type == MOT_LINE) {
    Serial.print(m.rapid ? "G0" : "G1");
    Serial.print(" X"); Serial.print(m.p1.x, 4);
    Serial.print(" Y"); Serial.print(m.p1.y, 4);
    Serial.print("\n");
    return;
  }

  if (m.type == MOT_ARC) {
    float I = m.center.x - m.p0.x;
    float J = m.center.y - m.p0.y;

    Serial.print((m.arcDir == ARC_CW) ? "G2" : "G3");
    Serial.print(" X"); Serial.print(m.p1.x, 4);
    Serial.print(" Y"); Serial.print(m.p1.y, 4);
    Serial.print(" I"); Serial.print(I, 4);
    Serial.print(" J"); Serial.print(J, 4);
    Serial.print("\n");
    return;
  }
}

static void process_one_gcode_line(const char* raw)
{
  char clean[160];
  strip_comments(raw, clean, sizeof(clean));

  const char* p = clean;
  while (*p == ' ' || *p == '\t') ++p;
  if (*p == 0) return;

  // Scan tokens so we can see G41/G42/G40 even if no motion
  ScanLine s;
  scan_line(clean, s);

  // Interpret to motion (also updates modal.comp, modal.motionG, etc.)
  Move2D mv = interpret_to_move(s, modal);

  // --- Handle comp transitions in the right order ---
  // If this block turns comp ON (G41/G42), enable comp BEFORE processing this move.
  if (ENABLE_COMP && (s.sawG41 || s.sawG42)) {
    cc.setComp(modal.comp);          // modal.comp is now LEFT/RIGHT
  }

  // If no motion, still might be a comp-toggle-only line; we’re done.
  if (mv.type == MOT_EMPTY) {
    // If someone ever sends G40 on a non-motion line, you could still turn it off here.
    if (ENABLE_COMP && s.sawG40) {
      cc.setComp(COMP_OFF);
      cc.flush();
    }
    return;
  }

  // Process either direct or through comp
  if (!ENABLE_COMP) {
    emit_move_as_gcode(mv);
  } else {
    if (!cc.pushIn(mv)) {
      Serial.println("(comp input buffer full)");
      return;
    }
    cc.process(true); // force rolling for demo; you could disable this for more VB-like behavior

    Move2D out;
    while (cc.popOut(out)) {
      emit_move_as_gcode(out);
    }
  }

  // If this block turns comp OFF (G40), disable comp AFTER processing this move.
  if (ENABLE_COMP && s.sawG40) {
    cc.setComp(COMP_OFF);
    cc.flush();
  }
}

static void flush_pipeline()
{
  if (!ENABLE_COMP) return;

  cc.flush();
  Move2D out;
  while (cc.popOut(out)) {
    emit_move_as_gcode(out);
  }
}


void setup()
{
  Serial.begin(BAUD);
  while (!Serial) { }

  Serial.println();
  Serial.println("Demo: TinyGCodeScan + CutterComp2D");
  Serial.print("Comp: "); Serial.println(ENABLE_COMP ? "ON" : "OFF");
  Serial.print("Fillet: "); Serial.println(ENABLE_FILLET ? "ON" : "OFF");
  Serial.print("Tool radius: "); Serial.println(TOOL_RADIUS, 6);
  Serial.println("--------------------------------------------------");
  Serial.println("Output is normalized motion G-code (G0/G1/G2/G3 with IJ).");
  Serial.println("--------------------------------------------------\n");

  // Init modal state
  modal = ModalState{};
  modal.planeXY = true;
  modal.absXY   = true;
  modal.motionG = 0;
  modal.comp    = COMP_OFF;
  modal.feed    = 0;
  modal.pos     = v2(0, 0);

  // Init cutter comp engine
  cc.setToolRadius(TOOL_RADIUS);
  cc.setCornerRolling(ENABLE_FILLET);
  cc.setComp(COMP_OFF);

  // Run demo program once
  const int lines = (int)(sizeof(demo_program) / sizeof(demo_program[0]));
  for (int i = 0; i < lines; ++i) {
    process_one_gcode_line(demo_program[i]);
  }
  flush_pipeline();

  Serial.println("\n--------------------------------------------------");
  Serial.println("Done.");
}

void loop()
{
  // No repeating demo in loop. Add your real-time streaming here later.
}
