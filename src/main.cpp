#ifdef ARDUINO
#include <Arduino.h>
#else
#include "ArduinoCompat.h"
#endif

#ifdef ARDUINO
#include <Arduino.h>
#define DBG_PRINTLN(x) Serial.println(x)
#define DBG_PRINT(x) Serial.print(x)
//#define DBG_PRINT(x, ...) Serial.print(x, ##__VA_ARGS__)
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
static constexpr float TOOL_RADIUS = 0.00625f;
static constexpr CornerType CORNER_TREATMENT = CORNER_ROLL; // CORNER_ROLL or CORNER_CHAMFER
static constexpr bool PERFORM_TRIM = true;
static constexpr bool ENABLE_MERGE = true;
static constexpr int MAX_LOOKAHEAD = 10; // MaxLookaheadForIntersections
// Crossing search can inspect up to (src + 2 + MAX_LOOKAHEAD), so we must
// keep at least that many tail elements un-emitted between batches.
static constexpr int TRIM_OVERLAP_MOVES = MAX_LOOKAHEAD + 2;
static constexpr int EMIT_HOLDBACK = TRIM_OVERLAP_MOVES;
static_assert(EMIT_HOLDBACK >= TRIM_OVERLAP_MOVES,"EMIT_HOLDBACK must preserve trim overlap across batches");
// Throughput knob: independent from MAX_LOOKAHEAD correctness settings.
static constexpr int TARGET_BATCH_EMIT_MOVES = 20;
static constexpr int MIN_PENDING_BEFORE_BATCH = EMIT_HOLDBACK + TARGET_BATCH_EMIT_MOVES;
static_assert(TARGET_BATCH_EMIT_MOVES > 0, "TARGET_BATCH_EMIT_MOVES must be positive");
// ------------------------------------------------

static ModalState modalState;
static CutterComp2D cc;
static float activeToolRadius = TOOL_RADIUS;

// -------------------- Profile buffer for post-pass trimming --------------------
static constexpr int MAX_PROFILE_MOVES = 32; // bump if needed
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

const char* fmtVal(char buf[], float v, int precision = 4)
{
  snprintf(buf, 32, "%.*f", precision, v);
  return buf;
}

// -------------------- G-code emission --------------------
static void emit_move_as_gcode(const Move2D &m)
{
  char buf[32];
  if (m.type == MOT_LINE || m.type == MOT_RAPID)
  {
    DBG_PRINT("N");
    DBG_PRINT(m.seqNum);
    DBG_PRINT(m.type == MOT_RAPID ? " G0" : " G1");
    DBG_PRINT(" X");
    DBG_PRINT(fmtVal(buf, m.p_1.x, 4));
    DBG_PRINT(" Y");
    DBG_PRINT(fmtVal(buf, m.p_1.y, 4));
    if (m.hasZ)
    {
      DBG_PRINT(" Z");
      DBG_PRINT(fmtVal(buf, m.z_1, 4));
    }
    DBG_PRINT("\n");
  }

  if (m.type == MOT_ARC)
  {
    Vec2 dCenter = m.center - m.p_0;

    DBG_PRINT("N");
    DBG_PRINT(m.seqNum);
    DBG_PRINT((m.arcDir == ARC_CW) ? " G2" : " G3");
    DBG_PRINT(" X");
    DBG_PRINT(fmtVal(buf, m.p_1.x, 4));
    DBG_PRINT(" Y");
    DBG_PRINT(fmtVal(buf, m.p_1.y, 4));
    DBG_PRINT(" I");
    DBG_PRINT(fmtVal(buf, dCenter.x, 4));
    DBG_PRINT(" J");
    DBG_PRINT(fmtVal(buf, dCenter.y, 4));
    if (m.hasZ)
    {
      DBG_PRINT(" Z");
      DBG_PRINT(fmtVal(buf, m.z_1, 4));
    }
    DBG_PRINT("\n");
    return;
  }
}


static bool emit_comp_profile_delta(const Move2D *moves,
                                    int profileSize,
                                    int &nextEmitIndex,
                                    int holdBackCount,
                                    bool flushAll)
{
  if (nextEmitIndex < 0)
    nextEmitIndex = 0;

  const float minCompLen = (activeToolRadius < 0.0f) ? -activeToolRadius : activeToolRadius;

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

    if (m.compMode == CM_IN || m.compMode == CM_OUT)
    {
      float moveLen = 0.0f;
      if (m.type == MOT_ARC)
        moveLen = distFromStart_along(m, m.p_1);
      else
        moveLen = len(m.p_1 - m.p_0);

      if (moveLen <= minCompLen)
      {
        DBG_PRINT("(comp transition too short:)");
        return false;
      }
    }

    emit_move_as_gcode(m);
  }

  nextEmitIndex = emitLimit;
  return true;
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

  if (s.hasD)
  {
   //TODO: tool table lookup could go here if desired.
  }

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

static bool trim_and_merge_pending_profile(int emittedProfileCount, int &trimResumeIndex)
{
  const int currentProfileCount = profileCount;
  int trimStart = trimResumeIndex;
  if (trimStart < emittedProfileCount)
    trimStart = emittedProfileCount;

  if (trimStart >= currentProfileCount)
    return true;

  if (PERFORM_TRIM)
  {
    int srcIdx = trimStart;
    int retTargetIdx = -1;
    if (!cc.trimCrossingElements(profile, srcIdx, currentProfileCount, MAX_LOOKAHEAD, retTargetIdx))
      return false;

    int mergeStart = trimStart;
    if (mergeStart > emittedProfileCount)
      mergeStart -= 1;

    cc.merge_all_colinear(profile + mergeStart, currentProfileCount - mergeStart);
  }

  int nextTrimStart = currentProfileCount - TRIM_OVERLAP_MOVES;
  if (nextTrimStart < emittedProfileCount)
    nextTrimStart = emittedProfileCount;
  if (nextTrimStart < 0)
    nextTrimStart = 0;
  trimResumeIndex = nextTrimStart;
  return true;
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
  DBG_PRINTLN(CORNER_TREATMENT == CORNER_ROLL ? "ROLL" : "CHAMFER");
  DBG_PRINT("Crossing trim: ");
  DBG_PRINTLN(PERFORM_TRIM ? "ON" : "OFF");
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
  activeToolRadius = TOOL_RADIUS;
  cc.setCornerTreatment(CORNER_TREATMENT);
  cc.setComp(COMP_OFF);

  // Reset profile buffer
  profile_reset();

  int emittedProfileCount = 0;
  int trimResumeIndex = 0;
  bool sawCompStart = false;
  bool sawG40 = false;
  bool compClosed = false;

  // Run demo program once
  const int lines = (int)(sizeof(demo_program) / sizeof(demo_program[0]));
  for (int i = 0; i < lines; ++i)
  {
    // Peek at the line to detect comp transitions for comment output
    char peekClean[160];
    strip_comments(demo_program[i], peekClean, sizeof(peekClean));
    ScanLine peekS;
    scan_line(peekClean, peekS);

    const bool compIsOff = (cc.comp_state == COMP_OFF);
    const bool entersComp = compIsOff && (peekS.sawG41 || peekS.sawG42) && !compClosed;

    if (entersComp)
    {
      sawCompStart = true;
      DBG_PRINTLN("(comp start:)");
    }

    process_one_gcode_line(demo_program[i]);

    if (cc.comp_state != COMP_OFF)
    {
      const int pendingProfileWindow = profileCount - emittedProfileCount;
      if (pendingProfileWindow >= MIN_PENDING_BEFORE_BATCH)
      {
        if (!trim_and_merge_pending_profile(emittedProfileCount, trimResumeIndex))
        {
          Serial.println("(trim failed)");
          return;
        }

        if (!emit_comp_profile_delta(profile, profileCount, emittedProfileCount, EMIT_HOLDBACK, false))
        {
          Serial.println("(emit failed)");
          return;
        }
        DBG_PRINT("(comp batch emit)\n");
      }
    }

    if (cc.comp_state == COMP_OFF && sawCompStart && !compClosed)
    {
      sawG40 = true;
      compClosed = true;
      if (!trim_and_merge_pending_profile(emittedProfileCount, trimResumeIndex))
      {
        Serial.println("(trim failed)");
        return;
      }

      if (!emit_comp_profile_delta(profile, profileCount, emittedProfileCount, 0, true))
      {
        Serial.println("(emit failed)");
        return;
      }
      DBG_PRINTLN("(comp stop:)");
    }
  }

  if (sawCompStart && !compClosed)
  {
    flush_pipeline();

    if (!trim_and_merge_pending_profile(emittedProfileCount, trimResumeIndex))
    {
      Serial.println("(final trim failed)");
      return;
    }

    if (!emit_comp_profile_delta(profile, profileCount, emittedProfileCount, 0, true))
    {
      Serial.println("(final emit failed)");
      return;
    }
  }

  if (!sawG40)
    Serial.println("(warning: reached EOF before G40)");

  Serial.println("\n--------------------------------------------------");
  Serial.println("Done.");
}

void loop()
{
  // No repeating demo in loop. Add your real-time streaming here later.
}
