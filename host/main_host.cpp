#include <cstdio>
#include <vector>
#include <string>
#include <cstring>
#include <fstream>
#include <sstream>
#include <cctype>
// #include <algorithm>
// #include <cmath>
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

#include "SimpleGCodeScan.h"
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
static constexpr MachineType MACHINE_TYPE = MAC_MILL;

static constexpr float TOOL_RADIUS = 0.0625f;
static constexpr CornerType CORNER_TREATMENT = CORNER_CHAMFER;
//-------------------- Trimming config --------------------
static constexpr bool FULL_TRIM_CROSSINGS = false;
static constexpr int FULL_TRIM_MAX_PASSES = 1;//TODO: DO I NEED MORE THAN ONE PASS?
static constexpr int FULL_TRIM_MAX_LOOKAHEAD = 10;
//------------------- Incremental trimming config -------------------
static constexpr bool ENABLE_INCREMENTAL_TRIM = true;
static constexpr int TRIM_EVERY_PROCESSED_BLOCKS = 4;
static constexpr int MIN_BLOCKS_BEFORE_TRIM = 4;
static constexpr int MAX_TRIM_PASSES_PER_INCREMENTAL_RUN = 1;
// ------------------------------------------------

static ModalState modal;
static CutterComp2D cc;

// -------------------- Profile buffer --------------------
static std::vector<Move2D> profile;
static int profileCount = 0;
static int blocksSinceTrim = 0;
static void profile_reset()
{
  profile.clear();
  profileCount = 0;
  blocksSinceTrim = 0;
}

static void profile_push(const Move2D &m)
{
  Move2D t = m;
  t.valid = true;
  profile.push_back(t);
  profileCount++;
  blocksSinceTrim++;
}

static void maybe_trim_profile_incremental(bool forceRun)
{
  if constexpr (FULL_TRIM_CROSSINGS)
    return;

  if constexpr (!ENABLE_INCREMENTAL_TRIM)
    return;

  if ((int)profile.size() < MIN_BLOCKS_BEFORE_TRIM)
    return;

  if (!forceRun && blocksSinceTrim < TRIM_EVERY_PROCESSED_BLOCKS)
    return;

  const int profileSize = (int)profile.size();
  const int passLimit = forceRun ? FULL_TRIM_MAX_PASSES : MAX_TRIM_PASSES_PER_INCREMENTAL_RUN;

  for (int pass = 0; pass < passLimit; ++pass)
  {
    bool changed = cc.trimCrossingElements(profile.data(), profileSize, FULL_TRIM_MAX_LOOKAHEAD);
    if (!changed)
      break;
  }

  blocksSinceTrim = 0;
}

static inline void copy_gcode_line(char *dst, size_t dstSize, const char *src)
{
#ifdef _MSC_VER
  strncpy_s(dst, dstSize, src, _TRUNCATE);
#else
  std::strncpy(dst, src, dstSize - 1);
  dst[dstSize - 1] = '\0';
#endif
}

// -------------------- Pipeline --------------------
static bool process_one_gcode_line(const char *raw)
{
  char clean[160];
  strip_comments(raw, clean, sizeof(clean));

  const char *p = clean;
  while (*p == ' ' || *p == '\t')
    ++p;
  if (*p == 0)
    return true; // empty or comment-only line

  ScanLine s;
  scan_line(clean, s);

  Move2D mv = interpret_to_move(s, modal, MACHINE_TYPE);

#ifndef NDEBUG
  // copy raw line for testing only
  copy_gcode_line(mv.gcode_line, sizeof(mv.gcode_line), raw);
#endif

  if ((s.sawG41 || s.sawG42))
  {
    // comp mode is now LEFT/RIGHT; set in cutter comp BEFORE processing this move.
    cc.setComp(modal.comp);
  }

  if (cc.comp_state !=COMP_OFF && mv.compMode == CM_STEADY && mv.type == MOT_RAPID)
  {
    return false; // should never have rapid moves in the cutter compensation steady mode.
  }

  if (!cc.pushIn(mv))
  {
    std::puts("(comp input buffer full)");
    return false;
  }

  // main pump-----------------------------------------

  bool success = cc.process();
  if (!success)
  {
    std::puts("(comp processing failed)");
    success = false;
    return false;
  }
  //-------------------------------------------------

  Move2D out;
  while (cc.popOut(out))
    profile_push(out);
  maybe_trim_profile_incremental(false);

  if (s.sawG40)
  {
    cc.flush();
    cc.setComp(COMP_OFF); // cancel comp mode; flush any delayed moves through comp with comp OFF
    while (cc.popOut(out))
      profile_push(out);
    maybe_trim_profile_incremental(false);
  }
  return true;
}

static void flush_pipeline()
{
  cc.flush();
  Move2D out;
  while (cc.popOut(out))
    profile_push(out);
  maybe_trim_profile_incremental(true);
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
  m.absXYZ = true;
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
    Move2D mv = interpret_to_move(s, m, MACHINE_TYPE);
    if (mv.type != MOT_EMPTY)
    {
      mv.valid = true;
      orig.push_back(mv);
    }
  }
  return orig;
}

static std::string basename_no_ext(const std::string &path)
{
  size_t slash = path.find_last_of("/\\");
  size_t start = (slash == std::string::npos) ? 0 : (slash + 1);
  std::string name = path.substr(start);
  size_t dot = name.find_last_of('.');
  if (dot != std::string::npos)
    name = name.substr(0, dot);
  return name;
}

static std::string sanitize_radius_for_filename(float radius)
{
  char tmp[64];
  std::snprintf(tmp, sizeof(tmp), "%.6f", radius);
  std::string s(tmp);

  while (!s.empty() && s.back() == '0')
    s.pop_back();
  if (!s.empty() && s.back() == '.')
    s.pop_back();
  if (s.empty())
    s = "0";

  std::string out;
  out.reserve(s.size() + 2);
  out.push_back('r');
  for (char ch : s)
  {
    if (ch == '-')
      out.push_back('m');
    else if (ch == '.')
      out.push_back('p');
    else if (std::isalnum((unsigned char)ch))
      out.push_back(ch);
    else
      out.push_back('_');
  }
  return out;
}

[[maybe_unused]] static const char *corner_treatment_tag(CornerType cornerTreatment)
{
  if (cornerTreatment == CORNER_CHAMFER)
    return "chamfer";
  return "roll";
}

static bool run_profile(const std::vector<std::string> &program, float toolRadius, CornerType cornerTreatment = CORNER_ROLL)
{
  modal = ModalState{};
  modal.planeXY = true;
  modal.absXYZ = true;
  modal.motionG = 0;
  modal.comp = COMP_OFF;
  modal.feed = 0;
  modal.pos = v2(0, 0);

  cc = CutterComp2D{};
  cc.setToolRadius(toolRadius);
  cc.setMachineType(MACHINE_TYPE);
  cc.setCornerTreatment(cornerTreatment);
  cc.setComp(COMP_OFF);

  profile_reset();

  const int lines = (int)program.size();
  for (int i = 0; i < lines; ++i)
  {
    bool success = process_one_gcode_line(program[i].c_str());
    if (!success)
      break;
  }
  flush_pipeline();

  if constexpr (FULL_TRIM_CROSSINGS && !ENABLE_INCREMENTAL_TRIM)
  {
    for (int pass = 0; pass < FULL_TRIM_MAX_PASSES; ++pass)
    {
      bool changed = cc.trimCrossingElements(profile.data(), (int)profile.size(), FULL_TRIM_MAX_LOOKAHEAD);
      if (!changed)
        break;
    }
  }
  cc.merge_all_colinear(profile.data(), (int)profile.size());
  bool isChained = cc.is_profile_chained(profile.data(), (int)profile.size());
  return isChained;
  
}

int main()
{
  // const char *default_file = "../../data/RapidComp.nc";
  const char *default_file = "../../data/G41_1.nc";
  // const char *default_file = "../../data/G41_2.nc";
  // const char *default_file = "../../data/TortureTestG91.nc";
  // const char *default_file = "../../data/LatheDia.nc";
  // const char *default_file = "../../data/LatheRad.nc";
  // const char *default_file = "../../data/Sample2.nc";
  // const char *default_file = "../../data/ArcExtension_Test_ArcArc_1.nc";
  // const char *default_file = "../../data/TortureTestmm.nc";
  // const char *default_file = "../../data/simple1.nc";
  //const char *default_file = "../../data/TortureTestG90.nc";
  //const char *default_file = "../../data/AI_Torture.nc";
  // const char *default_file = "../../data/TortureTestSmallFilletsG91.nc";
  //const char *default_file = "../../data/SimplePockets.nc";
  const std::string inputFilePath(default_file);
  const std::string inputBaseName = basename_no_ext(inputFilePath);
  std::vector<std::string> program = load_program_from_file(default_file); // warm up file loading (for better timing when we print later)
  // std::vector<std::string> program = load_program_from_demo();

  if (program.empty())
    return 1;

  [[maybe_unused]] const float radiusStep = 0.0001f;
  [[maybe_unused]] const int maxRadiusIters = 200000;

  float toolRadius = TOOL_RADIUS;

  
  const CornerType cornerTreatment = CORNER_TREATMENT;
  [[maybe_unused]] const bool isvalid = run_profile(program, toolRadius, cornerTreatment);

  // Write outputs for testing/visualization
  auto orig = build_original_moves(program);

  const std::string radiusTag = sanitize_radius_for_filename(toolRadius);
  // const std::string outBaseName = std::string(corner_treatment_tag(cornerTreatment)) + "_" + radiusTag + "_" + inputBaseName;
  const std::string outBaseName = std::string("out");
  const std::string svgPath = "../../output/" + outBaseName + ".svg";
  const std::string ngcPath = "../../output/" + outBaseName + ".ngc";

  write_svg(svgPath.c_str(), profile, &orig, MACHINE_TYPE, false, true, fabs(toolRadius * 2.0f),
            false, true, false, inputBaseName.c_str(), toolRadius); // mirror for better visualization
  write_gcode(ngcPath.c_str(), profile, MACHINE_TYPE, toolRadius);

  std::printf("Wrote: %s, %s\n", svgPath.c_str(), ngcPath.c_str());
  return 0;
}
