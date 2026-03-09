#ifdef ARDUINO
#include <Arduino.h>
#else
#include "ArduinoCompat.h"
#endif

#include <string.h>

#ifdef ARDUINO
#include <Arduino.h>
#define DBG_PRINTLN(x) Serial.println(x)
#define DBG_PRINTFLN(x, n) Serial.println(x, n)
#define DBG_PRINT(x) Serial.print(x)
// #define DBG_PRINT(x, ...) Serial.print(x, ##__VA_ARGS__)
#else
#include <cstdio>
#define DBG_PRINTLN(x) std::puts(x)
#define DBG_PRINT(x) std::printf("%s", x)
#define DBG_PRINT(x, ...) std::printf(x, ##__VA_ARGS__)
#endif

#include "cc_simple_scan.h"
#include "cc_processor.h"
#include "test_data.h"

// -------------------- Config --------------------
static constexpr uint32_t BAUD = 115200;

// IMPORTANT: Tool radius must match the units of your G-code.
static constexpr float TOOL_RADIUS = 0.0625f;
static constexpr CornerType CORNER_TREATMENT = CORNER_ROLL; // CORNER_ROLL or CORNER_CHAMFER
static constexpr bool TRIM_CROSSING = true;
static constexpr bool OUTPUT_INCH_UNITS = true;

// Primary tuning knobs.
static constexpr int MAX_LOOKAHEAD = 10;           // MaxLookaheadForIntersections
static constexpr int TARGET_BATCH_EMIT_MOVES = 20; // Target number of moves to emit in each batch after trimming (independent from MAX_LOOKAHEAD, which sets the correctness bounds for trimming)
static constexpr int PROFILE_BURST_MARGIN = 2;     // Extra moves to allow in the profile buffer beyond the lookahead window before we stop accepting new moves (must be enough to hold the entire lookahead window plus any additional moves we want to batch emit at once)

// Derived sizing.
// Crossing search can inspect up to (src + 2 + MAX_LOOKAHEAD), so we must
// keep at least that many tail elements un-emitted between batches.
static constexpr int TRIM_OVERLAP_MOVES = MAX_LOOKAHEAD + 2;
static constexpr int EMIT_HOLDBACK = TRIM_OVERLAP_MOVES;
static constexpr int MIN_PENDING_BEFORE_BATCH = EMIT_HOLDBACK + TARGET_BATCH_EMIT_MOVES;
static constexpr int MAX_PROFILE_MOVES = MIN_PENDING_BEFORE_BATCH + PROFILE_BURST_MARGIN;

static_assert(TARGET_BATCH_EMIT_MOVES > 0, "TARGET_BATCH_EMIT_MOVES must be positive");
static_assert(EMIT_HOLDBACK >= TRIM_OVERLAP_MOVES, "EMIT_HOLDBACK must preserve trim overlap across batches");
static_assert(MAX_PROFILE_MOVES > MIN_PENDING_BEFORE_BATCH, "MAX_PROFILE_MOVES must exceed batch threshold");
// ------------------------------------------------

static constexpr int POS_DIGITS_INCH = 4;
static constexpr int POS_DIGITS_MM = 3;

static ModalState modalState;
static CutterComp2D cc;
static float activeToolRadius = TOOL_RADIUS;

static inline uint32_t now_us()
{
  return (uint32_t)micros();
}

static inline uint32_t elapsed_us(uint32_t startUs)
{
  return (uint32_t)(now_us() - startUs);
}

static void compErrorHandler(CompError err, uint32_t seqNum)
{
#ifdef ARDUINO
  Serial.print("CompError ");
  Serial.print((int)err);
  Serial.print(" N");
  Serial.println(seqNum);
#else
  std::printf("CompError %u N%u\n", (unsigned)err, (unsigned)seqNum);
#endif
}

// -------------------- Profile buffer for post-pass trimming --------------------
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

static void profile_compact(int &emittedProfileCount, int &trimResumeIndex)
{
  if (emittedProfileCount <= 0)
    return;

  const int dropCount = emittedProfileCount;
  const int keepCount = profileCount - dropCount;

  if (keepCount > 0)
    memmove(profile, profile + dropCount, (size_t)keepCount * sizeof(Move2D));

  profileCount = keepCount;
  emittedProfileCount = 0;

  trimResumeIndex -= dropCount;
  if (trimResumeIndex < 0)
    trimResumeIndex = 0;
}

static inline int emit_pos_digits()
{
  return OUTPUT_INCH_UNITS ? POS_DIGITS_INCH : POS_DIGITS_MM;
}

static void trim_trailing_zeros(char *s)
{
  char *dot = strchr(s, '.');
  if (!dot)
    return;

  char *end = s + strlen(s) - 1;
  while (end > dot && *end == '0')
  {
    *end = '\0';
    --end;
  }

  if (end == dot)
    *end = '\0';
}

static int append_text(char *line, size_t cap, int n, const char *txt)
{
  if (n < 0 || n >= (int)cap)
    return n;
  int wrote = snprintf(line + n, cap - (size_t)n, "%s", txt);
  if (wrote < 0)
    return n;
  return n + wrote;
}

static int append_coord(char *line, size_t cap, int n, const char *axis, float value, int digits)
{
  char num[32];
  int wrote = snprintf(num, sizeof(num), "%.*f", digits, value);
  if (wrote < 0)
    return n;
  trim_trailing_zeros(num);

  n = append_text(line, cap, n, " ");
  n = append_text(line, cap, n, axis);
  n = append_text(line, cap, n, num);
  return n;
}

// -------------------- G-code emission --------------------
static void emit_move_as_gcode(const Move2D &m)
{
  char line[160];
  int n = 0;
  const int posDigits = emit_pos_digits();
  static bool hasLastFeed = false;
  static float lastFeed = 0.0f;

  if (m.type == MOT_LINE || m.type == MOT_RAPID)
  {
    n = snprintf(line, sizeof(line), "%s", (m.type == MOT_RAPID) ? "G0" : "G1");

    if (m.hasXY)
    {
      n = append_coord(line, sizeof(line), n, "X", m.p_1.x, posDigits);
      n = append_coord(line, sizeof(line), n, "Y", m.p_1.y, posDigits);
    }

    if (m.feed > 0.0f && (!hasLastFeed || m.feed != lastFeed))
    {
      n = append_coord(line, sizeof(line), n, "F", m.feed, posDigits);
      lastFeed = m.feed;
      hasLastFeed = true;
    }

    if (m.hasZ)
      n = append_coord(line, sizeof(line), n, "Z", m.z_1, posDigits);

    n = append_text(line, sizeof(line), n, "\n");
  }
  else if (m.type == MOT_ARC)
  {
    Vec2 dCenter = m.center - m.p_0;
    n = snprintf(line, sizeof(line), "%s",(m.arcDir == ARC_CW) ? "G2" : "G3");

    n = append_coord(line, sizeof(line), n, "X", m.p_1.x, posDigits);
    n = append_coord(line, sizeof(line), n, "Y", m.p_1.y, posDigits);
    n = append_coord(line, sizeof(line), n, "I", dCenter.x, posDigits);
    n = append_coord(line, sizeof(line), n, "J", dCenter.y, posDigits);

    if (m.feed > 0.0f && (!hasLastFeed || m.feed != lastFeed))
    {
      n = append_coord(line, sizeof(line), n, "F", m.feed, posDigits);
      lastFeed = m.feed;
      hasLastFeed = true;
    }

    if (m.hasZ)
      n = append_coord(line, sizeof(line), n, "Z", m.z_1, posDigits);

    n = append_text(line, sizeof(line), n, "\n");
  }

  if (n > 0)
    Serial.write((const uint8_t *)line, (size_t)n);
}

static bool emit_comp_profile(const Move2D *moves,
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

// -------------------- Per-line processing --------------------
static void process_one_gcode_line(ScanLine s)
{
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

  if (TRIM_CROSSING)
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
  uint32_t runStartUs = now_us();

  Serial.begin(BAUD);
  while (!Serial)
  {
  }

  Serial.println();
  Serial.println("Demo: TinyGCodeScan + CutterComp2D + PostTrimCrossings");

  DBG_PRINT("Fillet: ");
  DBG_PRINTLN(CORNER_TREATMENT == CORNER_ROLL ? "ROLL" : "CHAMFER");
  DBG_PRINT("Crossing trim: ");
  DBG_PRINTLN(TRIM_CROSSING ? "ON" : "OFF");
  DBG_PRINT("Tool radius: ");
  DBG_PRINTFLN(TOOL_RADIUS, 5);
  DBG_PRINT("Lookahead: ");
  DBG_PRINTLN(MAX_LOOKAHEAD);
  DBG_PRINT("Batch emit target: ");
  DBG_PRINTLN(TARGET_BATCH_EMIT_MOVES);
  DBG_PRINT("Burst margin: ");
  DBG_PRINTLN(PROFILE_BURST_MARGIN);

  // Init modal state
  modalState = ModalState{};
  modalState.planeXY = true;
  modalState.absXYZ = true;
  modalState.motionG = 0;
  modalState.comp = COMP_OFF;
  modalState.feed = 0;
  modalState.speed = 0;
  modalState.pos = v2(0, 0);

  // Init cutter comp engine
  cc.setToolRadius(TOOL_RADIUS);
  activeToolRadius = TOOL_RADIUS;
  cc.setCornerTreatment(CORNER_TREATMENT);
  cc.setErrorCallback(compErrorHandler);
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
    // is empty line then skip early
    if (peekClean[0] == 0)
      continue;

    ScanLine scanLn;
    scan_line(peekClean, scanLn);

    const bool compIsOff = (cc.comp_state == COMP_OFF);
    const bool entersComp = compIsOff && (scanLn.sawG41 || scanLn.sawG42) && !compClosed;

    if (entersComp)
    {
      sawCompStart = true;
      DBG_PRINTLN("(comp start:)");
    }

 
    process_one_gcode_line(scanLn);

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

        if (!emit_comp_profile(profile, profileCount, emittedProfileCount, EMIT_HOLDBACK, false))
        {
          Serial.println("(emit failed)");
          return;
        }

        profile_compact(emittedProfileCount, trimResumeIndex);
        //DBG_PRINT("(comp batch emit)\n");
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

      if (!emit_comp_profile(profile, profileCount, emittedProfileCount, 0, true))
      {
        Serial.println("(emit failed)");
        return;
      }

      profile_compact(emittedProfileCount, trimResumeIndex);
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

    if (!emit_comp_profile(profile, profileCount, emittedProfileCount, 0, true))
    {
      Serial.println("(final emit failed)");
      return;
    }

    profile_compact(emittedProfileCount, trimResumeIndex);
  }

  if (!sawG40)
    Serial.println("(warning: reached EOF before G40)");

  uint32_t totalUs = elapsed_us(runStartUs);
  char totalSecBuf[24];
  uint32_t totalSec = totalUs / 1000000UL;
  uint32_t totalMs = (totalUs % 1000000UL) / 1000UL;
  snprintf(totalSecBuf, sizeof(totalSecBuf), "%lu.%03lu", (unsigned long)totalSec, (unsigned long)totalMs);
  DBG_PRINT("  total time (s)=");
  DBG_PRINTLN(totalSecBuf);

  Serial.println("\n--------------------------------------------------");
  Serial.println("Done.");
}

void loop()
{
  // No repeating demo in loop. Add your real-time streaming here later.
}
