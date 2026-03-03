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
static constexpr float TOOL_RADIUS = 0.0225f;
static constexpr bool FORCE_ROLL_AROUND = true;
static constexpr bool FULL_TRIM_CROSSINGS = true;
static constexpr bool ENABLE_MERGE = true;
static constexpr int MAX_LOOKAHEAD = 25; // MaxLookaheadForIntersections
static constexpr int EMIT_HOLDBACK = 2;
static constexpr int FULL_TRIM_MAX_PASSES = 6;                  // safety cap
// ------------------------------------------------

static ModalState modalState;
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

static bool emit_comp_profile_delta(const Move2D *moves,
                                    int profileSize,
                                    int &nextEmitIndex,
                                    int holdBackCount,
                                    bool flushAll)
{
  if (nextEmitIndex < 0)
    nextEmitIndex = 0;

  int emitLimit = profileSize;
  if (!flushAll)
  {
    emitLimit = profileSize - holdBackCount;
    if (emitLimit < 0)
      emitLimit = 0;
  }

  if (nextEmitIndex >= emitLimit)
    return true;

  for (int i = nextEmitIndex; i < emitLimit; ++i)
  {
    const Move2D &m = moves[i];
    if (!m.valid || m.type == MOT_EMPTY)
      continue;
    emit_move_as_gcode(m);
  }

  nextEmitIndex = emitLimit;
  return true;
}

// -------------------- G-code emission --------------------
static void emit_move_as_gcode(const Move2D &m)
{
  Vec3 p1m = internal_xy_to_machine(m.p_1);

  if (m.type == MOT_LINE)
  {
    DBG_PRINT("N");
    DBG_PRINT(m.seqNum);
    DBG_PRINT(m.type == MOT_RAPID ? " G0" : " G1");
    DBG_PRINT(" X");
    DBG_PRINT(p1m.x, 4);
    DBG_PRINT(" Y");
    DBG_PRINT(p1m.y, 4);
    DBG_PRINT("\n");
    return;
  }

  if (m.type == MOT_ARC)
  {
    Vec2 dInternal = m.center - m.p_0;
    Vec3 dMachine = internal_delta_xy_to_machine(dInternal);

    DBG_PRINT("N");
    DBG_PRINT(m.seqNum);
    DBG_PRINT((m.arcDir == ARC_CW) ? " G2" : " G3");
    DBG_PRINT(" X");
    DBG_PRINT(p1m.x, 4);
    DBG_PRINT(" Y");
    DBG_PRINT(p1m.y, 4);
    DBG_PRINT(" I");
    DBG_PRINT(dMachine.x, 4);
    DBG_PRINT(" J");
    DBG_PRINT(dMachine.y, 4);
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
  Move2D mv = interpret_move(s, modalState);

  // If this block turns comp ON (G41/G42), enable comp BEFORE processing this move.
  if (s.sawG41 || s.sawG42)
  {
    cc.setComp(modalState.comp); // modal.comp is now LEFT/RIGHT
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
    cc.flush();
    cc.setComp(COMP_OFF);

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
      bool changed = cc.trimCrossingElements(profile,0, profileCount, MAX_LOOKAHEAD);
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
  modalState = ModalState{};
  modalState.planeXY = true;
  modalState.absXYZ = true;
  modalState.motionG = 0;
  modalState.comp = COMP_OFF;
  modalState.feed = 0;
  modalState.pos = v2(0, 0);

  // Init cutter comp engine
  cc.setToolRadius(TOOL_RADIUS);
  cc.setCornerTreatment(FORCE_ROLL_AROUND ? CORNER_ROLL : CORNER_CHAMFER);
  cc.setComp(COMP_OFF);

  // Reset profile buffer
  profile_reset();

  int emittedProfileCount = 0;
  int linesSinceEmit = 0;
  int trimResumeIndex = 0;
  int mergeResumeIndex = 0;

  // Run demo program once
  const int lines = (int)(sizeof(demo_program) / sizeof(demo_program[0]));
  for (int i = 0; i < lines; ++i)
  {
    process_one_gcode_line(demo_program[i]);

    if (cc.comp_state != COMP_OFF)
    {
      linesSinceEmit++;
      if (linesSinceEmit >= MAX_LOOKAHEAD)
      {
        if (FULL_TRIM_CROSSINGS)
        {
          cc.trimCrossingElements(profile, trimResumeIndex, profileCount, MAX_LOOKAHEAD);
          trimResumeIndex += MAX_LOOKAHEAD;
        }

        if (ENABLE_MERGE)
        {
          int mergeStart = (mergeResumeIndex > 0) ? (mergeResumeIndex - 1) : 0;
          int mergeCount = profileCount - mergeStart;
          if (mergeCount > (MAX_LOOKAHEAD + 1))
            mergeCount = (MAX_LOOKAHEAD + 1);
          int mergeEnd = mergeStart + mergeCount;

          if (mergeCount > 1)
            cc.merge_all_colinear(profile, mergeStart, mergeEnd);

          mergeResumeIndex += MAX_LOOKAHEAD;
        }

        if (!emit_comp_profile_delta(profile, profileCount, emittedProfileCount, EMIT_HOLDBACK, false))
        {
          Serial.println("(emit failed)");
          return;
        }

        linesSinceEmit = 0;
      }
    }
  }

  flush_pipeline();

  if (!emit_comp_profile_delta(profile, profileCount, emittedProfileCount, 0, true))
  {
    Serial.println("(final emit failed)");
    return;
  }

  Serial.println("\n--------------------------------------------------");
  Serial.println("Done.");
}

void loop()
{
  // No repeating demo in loop. Add your real-time streaming here later.
}
