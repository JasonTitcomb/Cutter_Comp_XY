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
static constexpr CornerType CORNER_TREATMENT = CORNER_ROLL; // CORNER_ROLL or CORNER_CHAMFER
static constexpr bool PERFORM_TRIM = true;                  // whether to perform trimming of moves after compensation (generally should be true to get correct results, but can be disabled for testing/debugging purposes)
static constexpr int MAX_LOOKAHEAD = 20;
static constexpr int EMIT_HOLDBACK = 2;

// ------------------------------------------------

static ModalState modalState;
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

static inline void copy_gcode_line(char *dst, size_t dstSize, const char *src)
{
#ifdef _MSC_VER
  strncpy_s(dst, dstSize, src, _TRUNCATE);
#else
  std::strncpy(dst, src, dstSize - 1);
  dst[dstSize - 1] = '\0';
#endif
}

static bool emit_comp_profile_delta(FILE *f,
                                    const std::vector<Move2D> &moves,
                                    int &nextEmitIndex,
                                    int holdBackCount,
                                    bool flushAll)
{
  const int profileSize = (int)moves.size();
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
    emit_move_as_gcode(f, m, MACHINE_TYPE);
  }

  nextEmitIndex = emitLimit;
  return true;
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

  Move2D mv = interpret_move(s, modalState, MACHINE_TYPE);

#ifndef NDEBUG
  // copy raw line for testing only
  copy_gcode_line(mv.gcode_line, sizeof(mv.gcode_line), raw);
#endif

  if ((s.sawG41 || s.sawG42))
  {
    // comp mode is now LEFT/RIGHT; set in cutter comp BEFORE processing this move.
    cc.setComp(modalState.comp);
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

  if (s.sawG40)
  {
    cc.flush();
    cc.setComp(COMP_OFF); // cancel comp mode; flush any delayed moves through comp with comp OFF

    while (cc.popOut(out))
      profile_push(out);
  }
  return true;
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
    Move2D mv = interpret_move(s, m, MACHINE_TYPE);
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
  out.push_back('R');
  for (char ch : s)
  {
    if (ch == '-')
      out.push_back('m');
    else if (ch == '.')
      out.push_back('_');
    else if (std::isalnum((unsigned char)ch))
      out.push_back(ch);
    else
      out.push_back('_');
  }
  return out;
}

static bool run_profile_streaming(const char *inputPath,
                                  const char *emitGcodePath,
                                  float toolRadius,
                                  CornerType cornerTreatment = CORNER_ROLL)
{
  modalState = ModalState{};
  modalState.planeXY = true;
  modalState.absXYZ = true;
  modalState.motionG = 0;
  modalState.comp = COMP_OFF;
  modalState.feed = 0;
  modalState.pos = v2(0, 0);

  cc = CutterComp2D{};
  cc.setToolRadius(toolRadius);
  cc.setMachineType(MACHINE_TYPE);
  cc.setCornerTreatment(cornerTreatment);
  cc.setComp(COMP_OFF);

  profile_reset();

  std::ifstream in(inputPath);
  if (!in)
  {
    std::fprintf(stderr, "Failed to open input file: %s\n", inputPath);
    return false;
  }

  bool sawCompStart = false;
  bool sawG40 = false;
  bool compClosed = false;
  int emittedProfileCount = 0;
  bool profileValid = true;
  int linesSinceEmit = 0;
  int trimResumeIndex = 0;
  int mergeResumeIndex = 0;
  std::string line;

  FILE *out = open_file_write_binary(emitGcodePath);
  if (!out)
  {
    std::fprintf(stderr, "Failed to open output file: %s\n", emitGcodePath);
    return false;
  }

  while (std::getline(in, line))
  {
    if (!line.empty() && line.back() == '\r')
      line.pop_back();

    char clean[160];
    strip_comments(line.c_str(), clean, sizeof(clean));
    ScanLine s;
    scan_line(clean, s);

    const bool compIsOff = (cc.comp_state == COMP_OFF);
    const bool entersComp = compIsOff && (s.sawG41 || s.sawG42) && !compClosed;

    if (entersComp)
    {
      sawCompStart = true;
      std::fprintf(out, "(comp start: %s)\n", s.sawG41 ? "G41" : "G42");
    }

    if (compIsOff && !entersComp)
    {
      interpret_move(s, modalState, MACHINE_TYPE);
      if (!s.sawG40)
        std::fprintf(out, "%s\n", line.c_str());// pass through unmodified until comp starts
      continue;
    }

    if (!process_one_gcode_line(line.c_str()))
    {
      std::fclose(out);
      return false;
    }

    // TODO: validate enough room to comp in.
    // If not, alarm.

    if (cc.comp_state != COMP_OFF)
    {
      linesSinceEmit++;
      if (linesSinceEmit >= MAX_LOOKAHEAD)
      {
        const int profileSize = (int)profile.size();

        if (PERFORM_TRIM)
        {
          cc.trimCrossingElements(profile.data(), trimResumeIndex, profileSize, MAX_LOOKAHEAD);
          trimResumeIndex += MAX_LOOKAHEAD;
        }

        int mergeStart = (mergeResumeIndex > 0) ? (mergeResumeIndex - 1) : 0;
        int mergeCount = profileSize - mergeStart;
        if (mergeCount > (MAX_LOOKAHEAD + 1))
          mergeCount = (MAX_LOOKAHEAD + 1);
        int mergeEnd = mergeStart + mergeCount;

        if (mergeCount > 1)
          cc.merge_all_colinear(profile.data(), mergeStart, mergeEnd);

        mergeResumeIndex += MAX_LOOKAHEAD;

        if (!emit_comp_profile_delta(out, profile, emittedProfileCount, EMIT_HOLDBACK, false))
        {
          std::fclose(out);
          return false;
        }
        linesSinceEmit = 0;
        std::fprintf(out, "(comp batch emit)\n");
      }

    }

    if (cc.comp_state == COMP_OFF && sawCompStart)
    {
      sawG40 = true;
      compClosed = true;

      if (!emit_comp_profile_delta(out, profile, emittedProfileCount, 0, true))
      {
        std::fclose(out);
        return false;
      }

      std::fprintf(out, "(comp stop: G40)\n");
    }
  }

  if (sawCompStart && !compClosed)
  {
    flush_pipeline();

    if (!emit_comp_profile_delta(out, profile, emittedProfileCount, 0, true))
    {
      std::fclose(out);
      return false;
    }
  }

  std::fclose(out);

  if (!sawG40)
    std::puts("(warning: reached EOF before G40)");

  if (sawCompStart && profile.empty())
    return false;

  if (sawCompStart)
  {
    const int profileSize = (int)profile.size();
    if constexpr (PERFORM_TRIM)
    {
      cc.trimCrossingElements(profile.data(), trimResumeIndex, profileSize, MAX_LOOKAHEAD);
    }

    int mergeStart = (mergeResumeIndex > 0) ? (mergeResumeIndex - 1) : 0;
    int mergeCount = profileSize - mergeStart;
    if (mergeCount > (MAX_LOOKAHEAD + 1))
      mergeCount = (MAX_LOOKAHEAD + 1);
    int mergeEnd = mergeStart + mergeCount;

    if (mergeCount > 1)
      cc.merge_all_colinear(profile.data(), mergeStart, mergeEnd);

  }

  return profileValid;
}

int main()
{
  // const char *default_file = "../../data/RapidComp.nc";
  // const char *default_file = "../../data/G41_1.nc";
  // const char *default_file = "../../data/G41_2.nc";
  // const char *default_file = "../../data/TortureTestG91.nc";
  // const char *default_file = "../../data/LatheDia.nc";
  // const char *default_file = "../../data/LatheRad.nc";
  // const char *default_file = "../../data/Sample2.nc";
  // const char *default_file = "../../data/ArcExtension_Test_ArcArc_1.nc";
  // const char *default_file = "../../data/TortureTestmm.nc";
  // const char *default_file = "../../data/simple1.nc";
   const char *default_file = "../../data/TortureTestG90.nc";
  // const char *default_file = "../../data/AI_Torture.nc";
  // const char *default_file = "../../data/TortureTestSmallFilletsG91.nc";
  //const char *default_file = "../../data/SimpleSquarePocket.nc";
  const std::string inputFilePath(default_file);
  const std::string inputBaseName = basename_no_ext(inputFilePath);

  float toolRadius = TOOL_RADIUS;

  const CornerType cornerTreatment = CORNER_TREATMENT;
  const std::string radiusTag = sanitize_radius_for_filename(toolRadius);
  // const std::string outBaseName = std::string(radiusTag + "_" + inputBaseName);
  const std::string outBaseName = std::string(inputBaseName);
  // const std::string outBaseName = std::string("out");
  const std::string svgPath = "../../output/" + outBaseName + ".svg";
  const std::string ngcPath = "../../output/" + outBaseName + ".ngc";

  const bool isvalid = run_profile_streaming(default_file, ngcPath.c_str(), toolRadius, cornerTreatment);
  if (!isvalid)
    std::puts("(warning: profile validation failed)");

  std::vector<std::string> program = load_program_from_file(default_file);
  if (program.empty())
    return 1;
  auto orig = build_original_moves(program);

  write_svg(svgPath.c_str(), profile, &orig, MACHINE_TYPE, false, true, fabs(toolRadius * 2.0f),
            false, true, false, inputBaseName.c_str(), toolRadius); // mirror for better visualization

  std::printf("Wrote: %s, %s\n", svgPath.c_str(), ngcPath.c_str());
  return 0;
}
