#ifdef ARDUINO
#include <Arduino.h>
#else
#include "ArduinoCompat.h"
#endif

#ifdef ARDUINO
#include <Arduino.h>
#define DBG_PRINTLN(x) Serial.println(x)
#define DBG_PRINT(x) Serial.print(x)
#define DBG_PRINT(x, ...) Serial.print(x, ##__VA_ARGS__)
#else
#include <cstdio>
#define DBG_PRINTLN(x) std::puts(x)
#define DBG_PRINT(x) std::printf("%s", x)
#define DBG_PRINT(x, ...) std::printf(x, ##__VA_ARGS__)
#endif

#include "SimpleGCodeScan.h"
#include "CutterComp2D.h"
#include "TestData.h"

// -------------------- Config --------------------
static constexpr uint32_t BAUD = 115200;

// IMPORTANT: Tool radius must match the units of your G-code.
static constexpr float TOOL_RADIUS = 0.0625f;
static constexpr bool FORCE_ROLL_AROUND = true;
static constexpr bool FULL_TRIM_CROSSINGS = true;
static constexpr bool ENABLE_MERGE = true;
static constexpr MachineType MACHINE_TYPE = MAC_MILL;
static constexpr int FULL_TRIM_MAX_LOOKAHEAD = 25; // MaxLookaheadForIntersections
static constexpr int FULL_TRIM_MAX_PASSES = 6;                  // safety cap
// ------------------------------------------------

static ModalState modal;
static CutterComp2D cc;

// -------------------- Profile buffer for post-pass trimming --------------------
static constexpr int MAX_PROFILE_MOVES = 512; // bump if needed
static Move2D profile[MAX_PROFILE_MOVES];
static int profileCount = 0;

static void profile_reset()
{
  profileCount = 0;
}

static bool profile_push(const Move2D &m)
{
  if (profileCount >= MAX_PROFILE_MOVES)
    return false;
  Move2D t = m;
  t.valid = true;
  profile[profileCount++] = t;
  return true;
}

// -------------------- G-code emission --------------------
static void emit_move_as_gcode(const Move2D &m)
{
  const bool latheMode = machine_is_lathe(MACHINE_TYPE);
  Vec3 p1m = internal_xy_to_machine(m.p_1, MACHINE_TYPE);

  if (m.type == MOT_LINE)
  {
    DBG_PRINT("N");
    DBG_PRINT(m.seqNum);
    DBG_PRINT(m.type == MOT_RAPID ? " G0" : " G1");
    DBG_PRINT(" X");
    DBG_PRINT(p1m.x, 4);
    DBG_PRINT(latheMode ? " Z" : " Y");
    DBG_PRINT(latheMode ? p1m.z : p1m.y, 4);
    DBG_PRINT("\n");
    return;
  }

  if (m.type == MOT_ARC)
  {
    Vec2 dInternal = m.center - m.p_0;
    Vec3 dMachine = internal_delta_xy_to_machine(dInternal, MACHINE_TYPE);

    DBG_PRINT("N");
    DBG_PRINT(m.seqNum);
    DBG_PRINT((m.arcDir == ARC_CW) ? " G2" : " G3");
    DBG_PRINT(" X");
    DBG_PRINT(p1m.x, 4);
    DBG_PRINT(latheMode ? " Z" : " Y");
    DBG_PRINT(latheMode ? p1m.z : p1m.y, 4);
    DBG_PRINT(" I");
    DBG_PRINT(dMachine.x, 4);
    DBG_PRINT(latheMode ? " K" : " J");
    DBG_PRINT(latheMode ? dMachine.z : dMachine.y, 4);
    DBG_PRINT("\n");
    return;
  }
}

// -------------------- Per-line processing --------------------
static void process_one_gcode_line(const char *raw)
{
  char clean[160];
  strip_comments(raw, clean, sizeof(clean));

  const char *p = clean;
  while (*p == ' ' || *p == '\t')
    ++p;
  if (*p == 0)
    return;

  // Scan tokens so we can see G41/G42/G40 even if no motion
  ScanLine s;
  scan_line(clean, s);

  // Interpret to motion (also updates modal.comp, modal.motionG, etc.)
  Move2D mv = interpret_to_move(s, modal, MACHINE_TYPE);

  // If this block turns comp ON (G41/G42), enable comp BEFORE processing this move.
  if (s.sawG41 || s.sawG42)
  {
    cc.setComp(modal.comp); // modal.comp is now LEFT/RIGHT
  }

  // No motion? still might be a comp-toggle-only line.
  if (mv.type == MOT_EMPTY)
  {
    // If someone ever sends G40 on a non-motion line, disable comp here.
    if (s.sawG40)
    {
      cc.setComp(COMP_OFF);
      cc.flush();
      Move2D out;
      while (cc.popOut(out))
      {
        if (!profile_push(out))
        {
          Serial.println("(profile buffer full)");
          return;
        }
      }
    }
    return;
  }

  if (!cc.pushIn(mv))
  {
    Serial.println("(comp input buffer full)");
    return;
  }

 
  // main pump-----------------------------------------
  bool success = cc.process();
  if (!success)
  {
    Serial.println("(comp processing failed due to insufficient output buffer space)");
    return;
  }
  //-------------------------------------------------


  Move2D out;
  while (cc.popOut(out))
  {
    if (!profile_push(out))
    {
      Serial.println("(profile buffer full)");
      return;
    }
  }

  // If this block turns comp OFF (G40), disable comp AFTER processing this move.
  if (s.sawG40)
  {
    // cc.setComp(COMP_OFF);
    cc.flush();

    Move2D out;
    while (cc.popOut(out))
    {
      if (!profile_push(out))
      {
        Serial.println("(profile buffer full)");
        return;
      }
    }
  }
}

static void flush_pipeline()
{
  cc.flush();
  Move2D out;
  while (cc.popOut(out))
  {
    if (!profile_push(out))
    {
      Serial.println("(profile buffer full)");
      return;
    }
  }
}

// --------------------  post pass: trim crossings --------------------
static void post_trim_and_merge()
{
  bool any = false;
  if (FULL_TRIM_CROSSINGS)
  {

    for (int pass = 0; pass < FULL_TRIM_MAX_PASSES; ++pass)
    {
      bool changed = cc.trimCrossingElements(profile, profileCount, FULL_TRIM_MAX_LOOKAHEAD);
      if (!changed)
      {
        DBG_PRINTLN("No crossings found on pass " + String(pass));
        break;
      }
      any = true;
    }
  }

  if (ENABLE_MERGE)
  {
    cc.merge_all_colinear(profile, profileCount);
  }

  
  DBG_PRINT("(post-trim crossings: ");
  DBG_PRINT(any ? "YES" : "NO");
  DBG_PRINTLN(")");
}

// -------------------- Arduino setup/loop --------------------
void setup()
{
  Serial.begin(BAUD);
  while (!Serial)
  {
  }

  Serial.println();
  Serial.println("Demo: TinyGCodeScan + CutterComp2D + PostTrimCrossings");
  
  DBG_PRINT("Fillet: ");
  DBG_PRINTLN(FORCE_ROLL_AROUND ? "ON" : "OFF");
  DBG_PRINT("Crossing trim: ");
  DBG_PRINTLN(FULL_TRIM_CROSSINGS ? "ON" : "OFF");
  DBG_PRINT("Tool radius: ");
  DBG_PRINTLN(TOOL_RADIUS);
  DBG_PRINT("Profile buffer cap: ");
  DBG_PRINTLN(MAX_PROFILE_MOVES);

  // Init modal state
  modal = ModalState{};
  modal.planeXY = true;
  modal.absXYZ = true;
  modal.motionG = 0;
  modal.comp = COMP_OFF;
  modal.feed = 0;
  modal.pos = v2(0, 0);

  // Init cutter comp engine
  cc.setToolRadius(TOOL_RADIUS);
  cc.setMachineType(MACHINE_TYPE);
  cc.setCornerTreatment(FORCE_ROLL_AROUND ? CORNER_ROLL : CORNER_CHAMFER);
  cc.setComp(COMP_OFF);

  // Reset profile buffer
  profile_reset();

  // Run demo program once
  const int lines = (int)(sizeof(demo_program) / sizeof(demo_program[0]));
  for (int i = 0; i < lines; ++i)
  {
    process_one_gcode_line(demo_program[i]);
  }
  flush_pipeline();

  // VB-style: remove offset self-crossing loops after full profile is built
  post_trim_and_merge();

  // Emit final profile
  for (int i = 0; i < profileCount; ++i)
  {
    if (!profile[i].valid)
      continue;
    if (profile[i].type == MOT_EMPTY)
      continue;
    emit_move_as_gcode(profile[i]);
  }

  Serial.println("\n--------------------------------------------------");
  Serial.println("Done.");
}

void loop()
{
  // No repeating demo in loop. Add your real-time streaming here later.
}
