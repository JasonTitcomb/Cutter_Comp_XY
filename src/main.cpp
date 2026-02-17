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

#include "TinyGCodeScan.h"
#include "CutterComp2D.h"
#include "TestData.h"

// -------------------- Config --------------------
static constexpr uint32_t BAUD = 115200;

// IMPORTANT: Tool radius must match the units of your G-code.
static constexpr float TOOL_RADIUS = 0.0625f;

// Toggle compensation on/off for A/B testing
static constexpr bool ENABLE_COMP = true;
static constexpr bool ENABLE_ROLL_AROUND = true;

static constexpr bool ENABLE_TRIM_CROSSINGS = true;
static constexpr int MAX_LOOKAHEAD_FOR_INTERSECTIONS = 25; // MaxLookaheadForIntersections
static constexpr int MAX_TRIM_PASSES = 6;                  // safety cap
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
  if (m.type == MOT_LINE)
  {
    DBG_PRINT("N");
    DBG_PRINT(m.seqNum);
    DBG_PRINT(m.rapid ? " G0" : " G1");
    DBG_PRINT(" X");
    DBG_PRINT(m.p1.x, 4);
    DBG_PRINT(" Y");
    DBG_PRINT(m.p1.y, 4);
    DBG_PRINT("\n");
    return;
  }

  if (m.type == MOT_ARC)
  {
    float I = m.center.x - m.p0.x;
    float J = m.center.y - m.p0.y;

    DBG_PRINT("N");
    DBG_PRINT(m.seqNum);
    DBG_PRINT((m.arcDir == ARC_CW) ? " G2" : " G3");
    DBG_PRINT(" X");
    DBG_PRINT(m.p1.x, 4);
    DBG_PRINT(" Y");
    DBG_PRINT(m.p1.y, 4);
    DBG_PRINT(" I");
    DBG_PRINT(I, 4);
    DBG_PRINT(" J");
    DBG_PRINT(J, 4);
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
  Move2D mv = interpret_to_move(s, modal);

  // If this block turns comp ON (G41/G42), enable comp BEFORE processing this move.
  if (ENABLE_COMP && (s.sawG41 || s.sawG42))
  {
    cc.setComp(modal.comp); // modal.comp is now LEFT/RIGHT
  }

  // No motion? still might be a comp-toggle-only line.
  if (mv.type == MOT_EMPTY)
  {
    // If someone ever sends G40 on a non-motion line, disable comp here.
    if (ENABLE_COMP && s.sawG40)
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

  // Push either direct or through comp, but ALWAYS store into profile[] for post-pass trimming.
  if (!ENABLE_COMP)
  {
    if (!profile_push(mv))
    {
      Serial.println("(profile buffer full)");
      return;
    }
  }
  else
  {
    if (!cc.pushIn(mv))
    {
      Serial.println("(comp input buffer full)");
      return;
    }

    // Force rolling (VB rollAround => forceRoll). You can change to false later.
    cc.process(ENABLE_ROLL_AROUND);

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

  // If this block turns comp OFF (G40), disable comp AFTER processing this move.
  if (ENABLE_COMP && s.sawG40)
  {
    //cc.setComp(COMP_OFF);
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
  if (!ENABLE_COMP)
    return;

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
static void post_trim_crossings()
{
  if (!ENABLE_TRIM_CROSSINGS)
    return;

  bool any = false;
  for (int pass = 0; pass < MAX_TRIM_PASSES; ++pass)
  {
    bool changed = cc.trimCrossingElements(profile, profileCount, MAX_LOOKAHEAD_FOR_INTERSECTIONS);
    if (!changed){
      Serial.println("No crossings found on pass " + String(pass));
      break;
    }
    any = true;
  }

  cc.fixup_comp_in_out(profile, profileCount);

  DBG_PRINT("(post-trim crossings: ");
  DBG_PRINT(any ? "YES" : "NO");
  Serial.println(")");
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
  DBG_PRINT("Comp: ");
  Serial.println(ENABLE_COMP ? "ON" : "OFF");
  DBG_PRINT("Fillet: ");
  Serial.println(ENABLE_ROLL_AROUND ? "ON" : "OFF");
  DBG_PRINT("Crossing trim: ");
  Serial.println(ENABLE_TRIM_CROSSINGS ? "ON" : "OFF");
  DBG_PRINT("Tool radius: ");
  Serial.println(TOOL_RADIUS, 6);
  DBG_PRINT("Profile buffer cap: ");
  Serial.println(MAX_PROFILE_MOVES);

  // Init modal state
  modal = ModalState{};
  modal.planeXY = true;
  modal.absXY = true;
  modal.motionG = 0;
  modal.comp = COMP_OFF;
  modal.feed = 0;
  modal.pos = v2(0, 0);

  // Init cutter comp engine
  cc.setToolRadius(TOOL_RADIUS);
  cc.setCornerRolling(ENABLE_ROLL_AROUND);
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
  post_trim_crossings();

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
