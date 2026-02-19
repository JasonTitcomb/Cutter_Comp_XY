#include <cstdio>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <cmath>
#include <iostream>

#define DBG_PRINTLN(x)             \
  do                               \
  {                                \
    std::cout << (x) << std::endl; \
  } while (0)
#define DBG_PRINT(x, ...) std::printf(x, ##__VA_ARGS__)

// If any of your headers include <Arduino.h>, include the compat first and
// make sure your headers include ArduinoCompat.h when not ARDUINO.
#include "ArduinoCompat.h"

#include "TinyGCodeScan.h"
#include "CutterComp2D.h"
#include "TestData.h"
#include "writer.h"
/*
  This is a desktop test harness for the CutterComp2D class, which performs 2D cutter compensation on linear and arc moves.

  It includes a post-pass to trim crossing elements, which is a common source of tiny unwanted moves after compensation. This is optional and can be toggled with ENABLE_TRIM_CROSSINGS.

  The test uses hardcoded G-code in TestData.h, which you can modify to test different scenarios. The output is printed as G-code lines, and also saved to "output.csv" for analysis and "output.svg" for visualization.

  Note: This code is meant for testing the compensation logic on the host. It does not run on an Arduino or control any hardware.
*/

// -------------------- Config --------------------
static constexpr float TOOL_RADIUS = 0.05f;
static constexpr bool ENABLE_ROLL_AROUND = true;
static constexpr bool ENABLE_TRIM_CROSSINGS = true;

static constexpr int MAX_LOOKAHEAD_FOR_INTERSECTIONS = 25;
static constexpr int MAX_TRIM_PASSES = 6;
// ------------------------------------------------

static ModalState modal;
static CutterComp2D cc;

// -------------------- Profile buffer --------------------
static std::vector<Move2D> profile;
static int profileCount = 0;
static void profile_reset()
{
  profile.clear();
  profileCount = 0;
}

static void profile_push(const Move2D &m)
{
  Move2D t = m;
  t.valid = true;
  profile.push_back(t);
  profileCount++;
}

// -------------------- Pipeline --------------------
static void process_one_gcode_line(const char *raw)
{
  char clean[160];
  strip_comments(raw, clean, sizeof(clean));

  const char *p = clean;
  while (*p == ' ' || *p == '\t')
    ++p;
  if (*p == 0)
    return;

  ScanLine s;
  scan_line(clean, s);

  Move2D mv = interpret_to_move(s, modal);

  // copy raw line for testing only
  strncpy(mv.gcode_line, raw, sizeof(mv.gcode_line) - 1);
  mv.gcode_line[sizeof(mv.gcode_line) - 1] = '\0';

  if ((s.sawG41 || s.sawG42))
  {
    // comp mode is now LEFT/RIGHT; set in cutter comp BEFORE processing this move.
    cc.setComp(modal.comp);
  }

  if (mv.compMode == CM_STEADY && mv.type == MOT_RAPID)
  {
    return; // should never have rapid moves in the cutter compensation steady mode.
  }


  if (!cc.pushIn(mv))
  {
    std::puts("(comp input buffer full)");
    return;
  }

  // main pump-----------------------------------------
  cc.process(ENABLE_ROLL_AROUND); // force rolling for demo
  //-------------------------------------------------

  Move2D out;
  while (cc.popOut(out))
    profile_push(out);

  if (s.sawG40)
  {
    cc.flush();
    cc.setComp(COMP_OFF); // cancel comp mode; flush any delayed moves through comp with comp OFF
    while (cc.popOut(out))
      profile_push(out);
  }
}

static void flush_pipeline()
{
  cc.flush();
  Move2D out;
  while (cc.popOut(out))
    profile_push(out);
}
static std::vector<std::string> load_program_from_file(const char *path)
{
  std::ifstream in(path);
  if (!in)
  {
    std::fprintf(stderr, "Failed to open input file: %s\n", path);
    return {};
  }

  std::vector<std::string> lines;
  std::string line;
  while (std::getline(in, line))
  {
    if (!line.empty() && line.back() == '\r')
      line.pop_back();
    lines.push_back(line);
  }
  return lines;
}


static std::vector<Move2D> build_original_moves(const std::vector<std::string> &program)
{
  std::vector<Move2D> orig;
  ModalState m{};
  m.planeXY = true;
  m.absXY = true;
  m.motionG = 0;
  m.comp = COMP_OFF;
  m.feed = 0;
  m.pos = v2(0, 0);

  for (const auto &line : program)
  {
    char clean[160];
    strip_comments(line.c_str(), clean, sizeof(clean));
    ScanLine s;
    scan_line(clean, s);
    Move2D mv = interpret_to_move(s, m);
    if (mv.type != MOT_EMPTY)
    {
      mv.valid = true;
      orig.push_back(mv);
    }
  }
  return orig;
}

int main()
{
  //const char *default_file = "../../data/RapidComp.nc";
  const char *default_file = "../../data/G41_2.nc";
  std::vector<std::string> program = load_program_from_file(default_file); // warm up file loading (for better timing when we print later)
  // std::vector<std::string> program = load_program_from_demo();

  if (program.empty())
    return 1;

  // Init modal
  modal = ModalState{};
  modal.planeXY = true;
  modal.absXY = true;
  modal.motionG = 0;
  modal.comp = COMP_OFF;
  modal.feed = 0;
  modal.pos = v2(0, 0);

  // Init cutter comp
  cc.setToolRadius(TOOL_RADIUS);
  cc.setCornerRolling(ENABLE_ROLL_AROUND);
  cc.setComp(COMP_OFF);

  profile_reset();

  const int lines = (int)program.size();
  for (int i = 0; i < lines; ++i)
  {
    process_one_gcode_line(program[i].c_str());
  }
  flush_pipeline();

  // Post-pass trim crossings (optional)
  if (ENABLE_TRIM_CROSSINGS)
  {
    for (int pass = 0; pass < MAX_TRIM_PASSES; ++pass)
    {
      bool changed = cc.trimCrossingElements(profile.data(), (int)profile.size(),MAX_LOOKAHEAD_FOR_INTERSECTIONS);
      if (!changed)
        break;
    }
  }



// Debug: show all moves in profile
for (size_t i = 0; i < profile.size(); i++) {
    const auto& m = profile[i];
    if (!m.valid) continue;
    const char* typeStr = (m.type == MOT_LINE) ? "LINE" : 
                          (m.type == MOT_ARC) ? "ARC" : 
                          (m.type == MOT_RAPID) ? "RAPID" : "EMPTY";
    std::printf("%2d: %s type=%d compMode=%d\n", (int)i, typeStr, m.type, (int)m.compMode);
}








  cc.merge_all_colinear(profile.data(), (int)profile.size());

  // Write outputs for testing/visualization
  auto orig = build_original_moves(program);

  write_svg("out.svg", profile, &orig, false, true, TOOL_RADIUS * 2.0f); // mirror for better visualization
  write_gcode("out.ngc", profile);

  std::puts("Wrote: out.svg, out.ngc, out.csv");
  return 0;
}
