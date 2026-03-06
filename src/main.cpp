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

#include "SimpleGCodeScan.h"
#include "CutterComp2D.h"
#include "TestData.h"

// -------------------- Config --------------------
static constexpr uint32_t BAUD = 115200;

// IMPORTANT: Tool radius must match the units of your G-code.
static constexpr float TOOL_RADIUS = 0.0625f;
static constexpr CornerType CORNER_TREATMENT = CORNER_ROLL; // CORNER_ROLL or CORNER_CHAMFER
static constexpr bool PERFORM_TRIM = true;
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

struct PerfStats
{
  uint32_t lines = 0;
  uint32_t batches = 0;
  uint32_t processCalls = 0;
  uint32_t trimCalls = 0;
  uint32_t emitCalls = 0;
  uint32_t compactCalls = 0;
  uint32_t flushCalls = 0;

  uint32_t processUs = 0;
  uint32_t trimUs = 0;
  uint32_t emitUs = 0;
  uint32_t compactUs = 0;
  uint32_t flushUs = 0;
};

static PerfStats perf;

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

    n = append_coord(line, sizeof(line), n, "X", m.p_1.x, posDigits);
    n = append_coord(line, sizeof(line), n, "Y", m.p_1.y, posDigits);

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
    // TODO: tool table lookup could go here if desired.
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
  perf = PerfStats{};
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
  DBG_PRINTLN(PERFORM_TRIM ? "ON" : "OFF");
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
    perf.lines++;

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

    uint32_t t0 = now_us();
    process_one_gcode_line(demo_program[i]);
    perf.processUs += elapsed_us(t0);
    perf.processCalls++;

    if (cc.comp_state != COMP_OFF)
    {
      const int pendingProfileWindow = profileCount - emittedProfileCount;
      if (pendingProfileWindow >= MIN_PENDING_BEFORE_BATCH)
      {
        t0 = now_us();
        if (!trim_and_merge_pending_profile(emittedProfileCount, trimResumeIndex))
        {
          Serial.println("(trim failed)");
          return;
        }
        perf.trimUs += elapsed_us(t0);
        perf.trimCalls++;

        t0 = now_us();
        if (!emit_comp_profile_delta(profile, profileCount, emittedProfileCount, EMIT_HOLDBACK, false))
        {
          Serial.println("(emit failed)");
          return;
        }
        perf.emitUs += elapsed_us(t0);
        perf.emitCalls++;

        t0 = now_us();
        profile_compact(emittedProfileCount, trimResumeIndex);
        perf.compactUs += elapsed_us(t0);
        perf.compactCalls++;
        perf.batches++;
        //DBG_PRINT("(comp batch emit)\n");
      }
    }

    if (cc.comp_state == COMP_OFF && sawCompStart && !compClosed)
    {
      sawG40 = true;
      compClosed = true;
      t0 = now_us();
      if (!trim_and_merge_pending_profile(emittedProfileCount, trimResumeIndex))
      {
        Serial.println("(trim failed)");
        return;
      }
      perf.trimUs += elapsed_us(t0);
      perf.trimCalls++;

      t0 = now_us();
      if (!emit_comp_profile_delta(profile, profileCount, emittedProfileCount, 0, true))
      {
        Serial.println("(emit failed)");
        return;
      }
      perf.emitUs += elapsed_us(t0);
      perf.emitCalls++;

      t0 = now_us();
      profile_compact(emittedProfileCount, trimResumeIndex);
      perf.compactUs += elapsed_us(t0);
      perf.compactCalls++;
      DBG_PRINTLN("(comp stop:)");
    }
  }

  if (sawCompStart && !compClosed)
  {
    uint32_t t0 = now_us();
    flush_pipeline();
    perf.flushUs += elapsed_us(t0);
    perf.flushCalls++;

    t0 = now_us();
    if (!trim_and_merge_pending_profile(emittedProfileCount, trimResumeIndex))
    {
      Serial.println("(final trim failed)");
      return;
    }
    perf.trimUs += elapsed_us(t0);
    perf.trimCalls++;

    t0 = now_us();
    if (!emit_comp_profile_delta(profile, profileCount, emittedProfileCount, 0, true))
    {
      Serial.println("(final emit failed)");
      return;
    }
    perf.emitUs += elapsed_us(t0);
    perf.emitCalls++;

    t0 = now_us();
    profile_compact(emittedProfileCount, trimResumeIndex);
    perf.compactUs += elapsed_us(t0);
    perf.compactCalls++;
  }

  if (!sawG40)
    Serial.println("(warning: reached EOF before G40)");

  DBG_PRINTLN("Timing (us):");
  DBG_PRINT("  lines=");
  DBG_PRINTLN(perf.lines);
  DBG_PRINT("  batches=");
  DBG_PRINTLN(perf.batches);

  DBG_PRINT("  process total=");
  DBG_PRINT(perf.processUs);
  DBG_PRINT(" avg=");
  DBG_PRINTLN(perf.processCalls ? (perf.processUs / perf.processCalls) : 0);

  DBG_PRINT("  trim total=");
  DBG_PRINT(perf.trimUs);
  DBG_PRINT(" avg=");
  DBG_PRINTLN(perf.trimCalls ? (perf.trimUs / perf.trimCalls) : 0);

  DBG_PRINT("  emit total=");
  DBG_PRINT(perf.emitUs);
  DBG_PRINT(" avg=");
  DBG_PRINTLN(perf.emitCalls ? (perf.emitUs / perf.emitCalls) : 0);

  DBG_PRINT("  compact total=");
  DBG_PRINT(perf.compactUs);
  DBG_PRINT(" avg=");
  DBG_PRINTLN(perf.compactCalls ? (perf.compactUs / perf.compactCalls) : 0);

  DBG_PRINT("  flush total=");
  DBG_PRINT(perf.flushUs);
  DBG_PRINT(" avg=");
  DBG_PRINTLN(perf.flushCalls ? (perf.flushUs / perf.flushCalls) : 0);

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
