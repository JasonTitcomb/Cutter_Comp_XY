#include <cstdio>
#include <vector>
#include <string>
#include <cstring>
#include <fstream>
#include <sstream>
#include <cctype>
#include <iostream>
#include <iomanip>
#include <iterator>
#if defined(__has_include)
#if __has_include(<filesystem>)
#include <filesystem>
#endif
#else
#include <filesystem>
#endif

#define DBG_PRINTLN(x)             \
  do                               \
  {                                \
    std::cout << (x) << std::endl; \
  } while (0)
#define DBG_PRINT(x, ...) std::printf(x, ##__VA_ARGS__)

#include "cc_simple_scan.h"
#include "cc_main.h"
#include "../mcu/cutter_comp_grblhal.h"
#include "writer.h"
#include "compare.h"
#include <cstdarg>
/*
  This is a desktop test harness for the CutterComp2D class, which performs 2D cutter compensation on linear and arc moves.

  It includes a post-pass to trim crossing elements, which is a common source of tiny unwanted moves after compensation. This is optional and can be toggled with ENABLE_TRIM_CROSSINGS.

  The test uses hardcoded G-code in test_data.h, which you can modify to test different scenarios. The output is printed as G-code lines, and also saved to "output.csv" for analysis and "output.svg" for visualization.

  Note: This code is meant for testing the compensation logic on the host. It does not run on an Arduino or control any hardware.
*/

// -------------------- Config --------------------

// Tool radius for cutter compensation override.
// If TOOL_RADIUS is set to 0.0f, the cutter compensation will use the tool radius in the file itself.
static constexpr float TOOL_RADIUS = 0;
static constexpr CornerType CORNER_TREATMENT = CORNER_ROLL; // CORNER_ROLL or CORNER_CHAMFER
static constexpr bool GLOBAL_TRIM_CROSSING = true;          // if true, will trim crossing elements down to the intersection point.
                                                            // If false, will emit the full compensated move even if it crosses.
static constexpr bool GLOBAL_MERGE = false;
static constexpr bool OUTPUT_SVG = true;

// ------------------------------------------------
static CutterComp2D cc;
struct HostRunnerContext
{
  FILE *out = nullptr;
  bool sawError = false;
};

static HostRunnerContext *g_hostRunnerContext = nullptr;
static std::string g_currentInputFile;

struct SweepDiagnostic
{
  bool sawError = false;
  unsigned code = 0;
  uint32_t line = 0;
  std::string message;
};

static SweepDiagnostic *g_sweepDiagnostic = nullptr;

static void host_output_cb(const char *text, size_t len)
{
  if (!g_hostRunnerContext || !g_hostRunnerContext->out || !text || len == 0)
    return;
  fwrite(text, 1, len, g_hostRunnerContext->out);
}

static void host_error_cb(const char *message, CompError err, uint32_t lineNum)
{
  if (g_sweepDiagnostic)
  {
    if (!g_sweepDiagnostic->sawError)
    {
      g_sweepDiagnostic->code = (unsigned)err;
      g_sweepDiagnostic->line = lineNum;
      g_sweepDiagnostic->message = message ? message : "Host compensation error";
    }
    g_sweepDiagnostic->sawError = true;
    return;
  }
  if (message)
    std::fprintf(stderr, "%s: %s\n", g_currentInputFile.c_str(), message);
  if (err != CE_ERROR)
    std::fprintf(stderr, "%s: CompMsg code=%u Ln%u\n", g_currentInputFile.c_str(), (unsigned)err, (unsigned)lineNum);
}

static void host_xy_error_cb(cc_status_code_t err, msg_type_t severity, uint32_t lineNum)
{
  if (g_sweepDiagnostic)
  {
    if (severity == CC_MSG_ERROR && err != cc_status_OK)
    {
      if (!g_sweepDiagnostic->sawError)
      {
        g_sweepDiagnostic->code = (unsigned)err;
        g_sweepDiagnostic->line = lineNum;
        g_sweepDiagnostic->message = cc_status_description(err);
      }
      g_sweepDiagnostic->sawError = true;
    }
    return;
  }
  (void)severity;
  if (err != cc_status_OK)
    std::fprintf(stderr, "%s: [mcu] CompMsg code=%u Ln%u\n", g_currentInputFile.c_str(), (unsigned)err, (unsigned)lineNum);
}

static std::vector<Move2D> *g_simpleProfileOut = nullptr;
static float g_mc_host_pos[N_AXIS] = {};

static bool host_cc_emit_via_mc(const move2d *mv)
{
  if (mv && mv->valid && mv->pause_after != 0.0f && g_simpleProfileOut)
  {
    Move2D marker{};
    marker.p_0 = v2(mv->p_0.x, mv->p_0.y);
    marker.p_1 = marker.p_0;
    marker.z_0 = mv->z_0;
    marker.z_1 = mv->z_1;
    marker.lineNum = mv->lineNum;
    marker.pause_after = mv->pause_after;
    marker.valid = true;
    g_simpleProfileOut->push_back(marker);
  }

  return cc_emit_via_mc(mv);
}

static gc_ccomp_t host_make_cc_state_for_side(CompSide side, bool enteringComp, float toolRadius)
{
  gc_ccomp_t ccState = {};

  if (side == COMP_LEFT)
    ccState.side = CComp_Left;
  else if (side == COMP_RIGHT)
    ccState.side = CComp_Right;
  else
    ccState.side = CComp_Off;

  ccState.first_move = enteringComp;
  ccState.radius = toolRadius;
  return ccState;
}

static cc_status_code_t cc_mc_line_arc_in_via_grblhal(const Move2D &mv, const gc_ccomp_t &ccState)
{
  plan_line_data_t pl_data = {};
  pl_data.feed_rate = mv.feed;
  pl_data.condition.rapid_motion = (mv.type == MOT_RAPID) ? 1 : 0;
  pl_data.line_number = mv.lineNum;

  float xyz[N_AXIS] = {mv.p_1.x, mv.p_1.y, mv.z_1};

  if (mv.type == MOT_ARC)
  {
    float position[N_AXIS] = {mv.p_0.x, mv.p_0.y, mv.z_0};
    float ijk[3] = {mv.center.x - mv.p_0.x, mv.center.y - mv.p_0.y, 0.0f};
    plane_t plane = {};
    plane.axis_0 = 0;
    plane.axis_1 = 1;
    plane.axis_linear = 2;
    int32_t turns = (mv.arcDir == ARC_CCW) ? mv.turns : -mv.turns;
    return cc_mc_arc_in(ccState, xyz, &pl_data, position, ijk, mv.radius, plane, turns);
  }

  return cc_mc_line_in(ccState, xyz, &pl_data);
}

extern "C"
{

  bool mc_line(float *xyz, plan_line_data_t *pl_data)
  {
    if (g_simpleProfileOut)
    {
      Move2D mv{};
      mv.p_0 = v2(g_mc_host_pos[0], g_mc_host_pos[1]);
      mv.p_1 = v2(xyz[0], xyz[1]);
      mv.z_0 = g_mc_host_pos[2];
      mv.z_1 = xyz[2];
      mv.feed = pl_data ? pl_data->feed_rate : 0.0f;
      mv.lineNum = pl_data ? pl_data->line_number : 0;
      mv.type = (pl_data && pl_data->condition.rapid_motion) ? MOT_RAPID : MOT_LINE;
      mv.valid = !(len(mv.p_1 - mv.p_0) < TOL && fabsf(mv.z_1 - mv.z_0) < TOL);
      g_simpleProfileOut->push_back(mv);
    }
    g_mc_host_pos[0] = xyz[0];
    g_mc_host_pos[1] = xyz[1];
    g_mc_host_pos[2] = xyz[2];
    return true;
  }

  void debug_printf(const char *fmt, ...)
  {
    if (g_sweepDiagnostic)
      return;
    char debug_out[100];

    va_list args;
    va_start(args, fmt);
    vsnprintf(debug_out, sizeof(debug_out) - 1, fmt, args);
    va_end(args);
    std::printf("(%s)\n", debug_out);
  }

  void report_message(const char *msg, message_type_t type)
  {
    if (g_sweepDiagnostic)
      return;
    (void)type;
    if (msg)
      std::printf("(%s)\n", msg);
  }

  void mc_arc(float *xyz, plan_line_data_t *pl_data, float *position, float *ijk, float radius, plane_t plane, int32_t turns)
  {
    (void)plane;
    if (g_simpleProfileOut)
    {
      Move2D mv{};
      mv.p_0 = v2(position[0], position[1]);
      mv.p_1 = v2(xyz[0], xyz[1]);
      mv.z_0 = position[2];
      mv.z_1 = xyz[2];
      mv.center = v2(position[0] + ijk[0], position[1] + ijk[1]);
      mv.radius = radius;
      mv.feed = pl_data ? pl_data->feed_rate : 0.0f;
      mv.lineNum = pl_data ? pl_data->line_number : 0;
      mv.type = MOT_ARC;
      mv.arcDir = (turns >= 0) ? ARC_CCW : ARC_CW;
      mv.valid = true;
      g_simpleProfileOut->push_back(mv);
    }
    g_mc_host_pos[0] = xyz[0];
    g_mc_host_pos[1] = xyz[1];
    g_mc_host_pos[2] = xyz[2];
  }
}

// -------------------- Profile buffer --------------------
static std::vector<Move2D> profile;
static int profileCount = 0;

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

static bool comp_starts_in_inches(const std::vector<std::string> &program)
{
  bool inchMode = false;
  for (const auto &line : program)
  {
    char clean[160];
    strip_comments(line.c_str(), clean, sizeof(clean));
    ScanLine scan;
    scan_line(clean, scan);
    if (scan.sawG20)
      inchMode = true;
    if (scan.sawG21)
      inchMode = false;
    if (scan.sawG41 || scan.sawG42)
      return inchMode;
  }
  return inchMode;
}

static float comp_tool_radius_from_diameter(const std::vector<std::string> &program)
{
  float diameter = 0.0f;
  for (const auto &line : program)
  {
    char clean[160];
    strip_comments(line.c_str(), clean, sizeof(clean));
    ScanLine scan;
    scan_line(clean, scan);
    if (scan.hasD)
      diameter = scan.D;
    if (scan.sawG41 || scan.sawG42)
      return diameter * 0.5f;
  }
  return 0.0f;
}

static ScanLine scan_in_mm(const ScanLine &source, bool inchMode);

static std::vector<Move2D> build_original_moves(const std::vector<std::string> &program, bool millimeters = false)
{
  std::vector<Move2D> orig;
  ModalState m{};
  m.planeXY = true;
  m.absoluteMode = true;
  m.motionG = 0;
  m.comp = COMP_OFF;
  m.feed = 0;
  m.pos = v2(0, 0);
  m.z = 0.0f;
  m.lineNumber = 0;

  uint32_t input_line_number = 1;
  for (const auto &line : program)
  {
    char clean[160];
    strip_comments(line.c_str(), clean, sizeof(clean));
    ScanLine s;
    scan_line(clean, s);
    m.lineNumber = input_line_number;
    set_physical_units(!millimeters && (s.sawG21 ? false : (s.sawG20 || m.inchMode)));
    Move2D mv = interpret_move(millimeters ? scan_in_mm(s, m.inchMode) : s, m);
    if (mv.type != MOT_EMPTY || mv.pauseKind != PAUSE_NONE)
    {
      mv.valid = true;
      orig.push_back(mv);
    }
    ++input_line_number;
  }
  return orig;
}

static ScanLine scan_in_mm(const ScanLine &source, bool inchMode)
{
  ScanLine scan = source;
  if (!source.sawG20 && (source.sawG21 || !inchMode))
    return scan;

  if (scan.hasX) scan.X *= 25.4f;
  if (scan.hasY) scan.Y *= 25.4f;
  if (scan.hasZ) scan.Z *= 25.4f;
  if (scan.hasI) scan.I *= 25.4f;
  if (scan.hasJ) scan.J *= 25.4f;
  if (scan.hasR) scan.R *= 25.4f;
  if (scan.hasF) scan.F *= 25.4f;
  return scan;
}

static bool run_profile_simple_xy(const std::vector<std::string> &program,
                                  float toolRadius,
                                  CornerType cornerTreatment,
                                  std::vector<Move2D> &profileOut,
                                  bool radiusInMillimeters = false)
{
  ModalState modal{};
  modal.planeXY = true;
  modal.absoluteMode = true;
  modal.motionG = 0;
  modal.comp = COMP_OFF;
  modal.feed = 0;
  modal.pos = v2(0, 0);
  modal.z = 0.0f;
  modal.inchMode = false;
  modal.lineNumber = 0;

  g_mc_host_pos[0] = 0.0f;
  g_mc_host_pos[1] = 0.0f;
  g_mc_host_pos[2] = 0.0f;
  cc_api_init(0.0f, CC_UNITS_MM, host_cc_emit_via_mc, host_xy_error_cb);
  cc_mc_sync_input_pos(g_mc_host_pos);

   g_simpleProfileOut = &profileOut;

  
  // Start reading the program line by line.
  size_t input_line_number = 0;
  for (const auto &line : program)
  {
    input_line_number++;
    char clean[160];
    strip_comments(line.c_str(), clean, sizeof(clean));

    ScanLine s;
    scan_line(clean, s);

    modal.lineNumber = input_line_number;
    const CompSide prevComp = modal.comp;
    set_physical_units(false);
    Move2D mv = interpret_move(scan_in_mm(s, modal.inchMode), modal);
    const bool enteringComp = (prevComp == COMP_OFF && modal.comp != COMP_OFF);
    const bool exitingComp = (prevComp != COMP_OFF && modal.comp == COMP_OFF);

    if (enteringComp)
    {
      cc_api_init(toolRadius, !radiusInMillimeters && modal.inchMode ? CC_UNITS_INCH : CC_UNITS_MM, host_cc_emit_via_mc, host_xy_error_cb);
      cc_api_set_lookahead_enabled(GLOBAL_TRIM_CROSSING);
      cc_api_set_corner_treatment_mode(cornerTreatment == CORNER_CHAMFER ? CC_CTM_CHAMFER : CC_CTM_ROLL);
    }

    if (mv.pauseKind != PAUSE_NONE)
    {
      if (cc_mc_enqueue_pause_marker(mv.pause_after) != cc_status_OK)
      {
        g_simpleProfileOut = nullptr;
        return false;
      }
      continue;
    }

    if (mv.type == MOT_EMPTY)
    {
      if (exitingComp)
      {
        if (cc_api_process_move(nullptr) != cc_status_OK)
        {
          g_simpleProfileOut = nullptr;
          return false;
        }
        cc_api_set_comp(CC_COMP_OFF);
      }
      continue;
    }

    gc_ccomp_t ccState = host_make_cc_state_for_side(modal.comp, enteringComp, toolRadius);

    // grblHAL rejects a G2/G3 P word that is not a positive integer before motion is issued.
    if (mv.type == MOT_ARC && mv.turns < 1)
    {
      host_xy_error_cb(cc_status_InvalidMove, CC_MSG_ERROR, mv.lineNum);
      g_simpleProfileOut = nullptr;
      return false;
    }

    if (cc_mc_line_arc_in_via_grblhal(mv, ccState) != cc_status_OK)
    {
      g_simpleProfileOut = nullptr;
      return false;
    }
  }

  // final move to flush any pending compensation moves through the system.
  if (cc_api_process_move(nullptr) != cc_status_OK)
  {
    g_simpleProfileOut = nullptr;
    return false;
  }

  g_simpleProfileOut = nullptr;
  return true;
}

static const char *motion_type_name(MotionType type)
{
  switch (type)
  {
  case MOT_RAPID:
    return "rapid";
  case MOT_LINE:
    return "line";
  case MOT_ARC:
    return "arc";
  default:
    return "empty";
  }
}

static float compare_vec_delta(const Vec2 &a, const Vec2 &b)
{
  return len(a - b);
}

static std::string lookup_gcode_line(const std::vector<std::string> &program, uint32_t lineNumber)
{
  if (lineNumber == 0)
    return "<line 0>";

  const size_t index = static_cast<size_t>(lineNumber - 1);
  if (index >= program.size())
    return "<line out of range>";

  return program[index];
}

static bool try_extract_n_word(const std::string &line, uint32_t &nWord)
{
  size_t i = 0;
  while (i < line.size())
  {
    while (i < line.size() && std::isspace(static_cast<unsigned char>(line[i])))
      ++i;

    if (i >= line.size())
      return false;

    if (line[i] == '(')
      return false;

    if (line[i] == 'N' || line[i] == 'n')
    {
      ++i;
      size_t start = i;
      while (i < line.size() && std::isdigit(static_cast<unsigned char>(line[i])))
        ++i;

      if (i > start)
      {
        nWord = static_cast<uint32_t>(std::strtoul(line.substr(start, i - start).c_str(), nullptr, 10));
        return true;
      }
      return false;
    }

    while (i < line.size() && !std::isspace(static_cast<unsigned char>(line[i])))
      ++i;
  }

  return false;
}

static std::vector<Move2D> filter_valid_moves(const std::vector<Move2D> &moves)
{
  std::vector<Move2D> filtered;
  filtered.reserve(moves.size());
  for (const Move2D &m : moves)
  {
    if (!m.valid)
      continue;
    filtered.push_back(m);
  }
  return filtered;
}

static std::vector<Move2D> filter_compare_moves(const std::vector<Move2D> &moves,
                                                const std::vector<std::string> &program)
{
  std::vector<Move2D> filtered;
  filtered.reserve(moves.size());

  for (const Move2D &m : moves)
  {
    if (!m.valid)
      continue;

    const bool hasXYMotion = len(m.p_1 - m.p_0) >= TOL_MM;

    const std::string gcodeLine = lookup_gcode_line(program, m.lineNum);
    if (gcodeLine.find("G53") != std::string::npos || gcodeLine.find("M30") != std::string::npos)
      continue;

    if (fabsf(m.z_1 - m.z_0) >= TOL_MM)
      continue;

    if (!hasXYMotion && m.type == MOT_RAPID)
      continue;

    filtered.push_back(m);
  }

  while (!filtered.empty() && filtered.back().type == MOT_RAPID)
    filtered.pop_back();

  return filtered;
}

static std::vector<Move2D> filter_compare_moves(const std::vector<Move2D> &moves)
{
  std::vector<Move2D> filtered;
  filtered.reserve(moves.size());

  for (const Move2D &m : moves)
  {
    if (!m.valid)
      continue;

    const bool hasXYMotion = len(m.p_1 - m.p_0) >= TOL_MM;
    if (fabsf(m.z_1 - m.z_0) >= TOL_MM)
      continue;

    if (!hasXYMotion && m.type == MOT_RAPID)
      continue;

    filtered.push_back(m);
  }

  while (!filtered.empty() && filtered.back().type == MOT_RAPID)
    filtered.pop_back();

  return filtered;
}

static size_t count_invalid_moves(const std::vector<Move2D> &moves)
{
  size_t count = 0;
  for (const Move2D &m : moves)
  {
    if (!m.valid)
      ++count;
  }
  return count;
}

static void write_xy_compare_report(const char *path,
                                    const std::vector<Move2D> &fullProfile,
                                    const std::vector<Move2D> &simpleProfile,
                                    const std::vector<std::string> &fullProgram,
                                    const std::vector<std::string> &simpleProgram,
                                    size_t invalidFullCount,
                                    size_t invalidSimpleCount)
{
  std::ofstream out(path);
  const float compareTol = 0.005f;
  const std::vector<ComparePair> pairs = align_compare_moves(fullProfile, simpleProfile, compareTol);
  size_t sharedCount = 0;
  size_t unmatchedFullCount = 0;
  size_t unmatchedSimpleCount = 0;
  for (const ComparePair &pair : pairs)
  {
    if (pair.fullIndex == fullProfile.size())
      ++unmatchedSimpleCount;
    else if (pair.simpleIndex == simpleProfile.size())
      ++unmatchedFullCount;
    else
      ++sharedCount;
  }
  size_t mismatchCount = 0;
  size_t typeMismatchCount = 0;
  size_t invalidMismatchCount = 0;
  int firstMismatch = -1;
  float maxP0Delta = 0.0f;
  float maxP1Delta = 0.0f;
  float maxCenterDelta = 0.0f;
  float maxRadiusDelta = 0.0f;
  if (!out)
  {
    std::fprintf(stderr, "Failed to open comparison report: %s\n", path);
    return;
  }

  out << "cc_xy comparison report\n";
  out << "full profile count: " << fullProfile.size() << "\n";
  out << "simple profile count: " << simpleProfile.size() << "\n";
  out << "invalid full count: " << invalidFullCount << "\n";
  out << "invalid simple count: " << invalidSimpleCount << "\n";
  out << "shared count: " << sharedCount << "\n";

  for (size_t i = 0; i < pairs.size(); ++i)
  {
    const ComparePair &pair = pairs[i];
    if (pair.fullIndex == fullProfile.size() || pair.simpleIndex == simpleProfile.size())
    {
      if (firstMismatch < 0)
        firstMismatch = (int)i;
      ++mismatchCount;
      if (mismatchCount <= 20)
      {
        const bool fullOnly = pair.simpleIndex == simpleProfile.size();
        const size_t index = fullOnly ? pair.fullIndex : pair.simpleIndex;
        const Move2D &move = fullOnly ? fullProfile[index] : simpleProfile[index];
        out << "\nindex " << i << " unmatched " << (fullOnly ? "full" : "simple")
            << " move (profile index " << index << ")\n"
            << "  type=" << motion_type_name(move.type) << " line=" << move.lineNum
            << " p0=(" << move.p_0.x << ", " << move.p_0.y << ")"
            << " p1=(" << move.p_1.x << ", " << move.p_1.y << ")\n"
            << "  gcode: " << lookup_gcode_line(fullOnly ? fullProgram : simpleProgram, move.lineNum) << "\n";
      }
      continue;
    }
    const Move2D &full = fullProfile[pair.fullIndex];
    const Move2D &simple = simpleProfile[pair.simpleIndex];
    const float p0Delta = compare_vec_delta(full.p_0, simple.p_0);
    const float p1Delta = compare_vec_delta(full.p_1, simple.p_1);
    const bool bothArcs = full.type == MOT_ARC && simple.type == MOT_ARC;
    const float centerDelta = bothArcs ? compare_vec_delta(full.center, simple.center) : 0.0f;
    const float radiusDelta = bothArcs ? fabsf(full.radius - simple.radius) : 0.0f;
    bool mismatch = false;

    if (p0Delta > maxP0Delta)
      maxP0Delta = p0Delta;
    if (p1Delta > maxP1Delta)
      maxP1Delta = p1Delta;
    if (centerDelta > maxCenterDelta)
      maxCenterDelta = centerDelta;
    if (radiusDelta > maxRadiusDelta)
      maxRadiusDelta = radiusDelta;

    if (full.type != simple.type)
    {
      ++typeMismatchCount;
      mismatch = true;
    }

    if (full.valid != simple.valid)
    {
      ++invalidMismatchCount;
      mismatch = true;
    }

    if (p0Delta > compareTol || p1Delta > compareTol)
      mismatch = true;

    if (bothArcs &&
        (full.arcDir != simple.arcDir || centerDelta > compareTol || radiusDelta > compareTol))
      mismatch = true;

    if (!mismatch)
      continue;

    if (firstMismatch < 0)
      firstMismatch = (int)i;

    ++mismatchCount;
    if (mismatchCount <= 20)
    {
      const std::string fullGcodeLine = lookup_gcode_line(fullProgram, full.lineNum);
      const std::string simpleGcodeLine = lookup_gcode_line(simpleProgram, simple.lineNum);
      uint32_t fullNWord = 0;
      uint32_t simpleNWord = 0;
      const bool fullHasNWord = try_extract_n_word(fullGcodeLine, fullNWord);
      const bool simpleHasNWord = try_extract_n_word(simpleGcodeLine, simpleNWord);

      out << "\nindex " << i << " mismatch (full index " << pair.fullIndex
          << ", simple index " << pair.simpleIndex << ")\n";
      out << "  full   : type=" << motion_type_name(full.type)
        << " out_line=" << full.lineNum
        << (fullHasNWord ? " n=" + std::to_string(fullNWord) : "")
          << " p0=(" << full.p_0.x << ", " << full.p_0.y << ")"
          << " p1=(" << full.p_1.x << ", " << full.p_1.y << ")"
          << " r=" << full.radius << " valid=" << full.valid << "\n";
      out << "  simple : type=" << motion_type_name(simple.type)
        << " src_line=" << simple.lineNum
        << (simpleHasNWord ? " n=" + std::to_string(simpleNWord) : "")
          << " p0=(" << simple.p_0.x << ", " << simple.p_0.y << ")"
          << " p1=(" << simple.p_1.x << ", " << simple.p_1.y << ")"
          << " r=" << simple.radius << " valid=" << simple.valid << "\n";
      out << "  full gcode  : " << fullGcodeLine << "\n";
      out << "  simple gcode: " << simpleGcodeLine << "\n";
      out << std::fixed << std::setprecision(6)
          << "  delta  : p0=" << p0Delta
          << " p1=" << p1Delta
          << " center=" << centerDelta
          << " radius=" << radiusDelta << "\n";
      out.unsetf(std::ios::floatfield);
    }
  }

  if (fullProfile.size() != simpleProfile.size())
  {
    out << "\ncount mismatch: full=" << fullProfile.size()
        << " simple=" << simpleProfile.size() << "\n";
  }

  out << "\nsummary\n";
  out << "  first mismatch index: " << firstMismatch << "\n";
  out << "  mismatch count: " << mismatchCount << "\n";
  out << "  unmatched full count: " << unmatchedFullCount << "\n";
  out << "  unmatched simple count: " << unmatchedSimpleCount << "\n";
  out << "  type mismatch count: " << typeMismatchCount << "\n";
  out << "  validity mismatch count: " << invalidMismatchCount << "\n";
  out << std::fixed << std::setprecision(6)
      << "  max p0 delta: " << maxP0Delta << "\n"
      << "  max p1 delta: " << maxP1Delta << "\n"
      << "  max center delta: " << maxCenterDelta << "\n"
      << "  max radius delta: " << maxRadiusDelta << "\n";
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

static bool run_profile_streaming(const char *inputPath,
                                  const char *emitGcodePath,
                                  float toolRadius,
                                  CornerType cornerTreatment = CORNER_ROLL)
{
  std::ifstream in(inputPath);
  if (!in)
  {
    std::fprintf(stderr, "Failed to open input file: %s\n", inputPath);
    return false;
  }

  FILE *outputFile = open_file_write_binary(emitGcodePath);
  if (!outputFile)
  {
    std::fprintf(stderr, "Failed to open output file: %s\n", emitGcodePath);
    return false;
  }

  HostRunnerContext ctx;

  ctx.out = outputFile;

  g_hostRunnerContext = &ctx;

  CcMainRunner runner;
  CcMainOptions options;
  options.toolRadius = toolRadius;
  options.cornerTreatment = cornerTreatment;
  options.globalTrimCrossing = GLOBAL_TRIM_CROSSING;
  options.globalMerge = GLOBAL_MERGE;
  options.callbacks.output = host_output_cb;
  options.callbacks.error = host_error_cb;

  bool ok = runner.begin(options);
  std::string line;
  while (ok && std::getline(in, line))
  {
    runner.incrementLineNumber();
    if (!line.empty() && line.back() == '\r')
      line.pop_back();

    ok = runner.processLine(line.c_str());
  }

  if (ok)
    ok = runner.finish();

  g_hostRunnerContext = nullptr;

  std::fclose(outputFile);
  return ok && !ctx.sawError;
}

#include "../test/cc_diameter_sweep.h"

// Command line usage:
// main_host [inputfile|all] [outputfolder] [toolradius] [cornerTreatment] [svg]
// Defaults:
//   inputfile: all files in default_files (or one file when supplied)
//   outputfolder: "../../output/"
//   toolradius: TOOL_RADIUS (in the input program's units)
//   cornerTreatment: CORNER_TREATMENT ("roll" or "chamfer")
//   svg: "svg" (default, output SVG), or "nosvg" (do not output SVG)
int main(int argc, char *argv[])
{
  const char *default_files[] = {
      "../../data/RapidComp.nc",
      "../../data/G41_1.nc",
      "../../data/ThreadMill.nc",
      "../../data/G41_2.nc",
      "../../data/TortureTestG91.nc",
      "../../data/Sample2.nc",
      "../../data/Sample3.nc",
      "../../data/Sample2mm.nc",
      "../../data/TortureTestmm.nc",
      "../../data/_TortureTestG90.nc",
      "../../data/TortureTestG90.nc",
      "../../data/TortureTestG90LARGE.nc",
      "../../data/TortureTestG90LARGE2X.nc",
      "../../data/TortureTestG90SMALL.nc",
      "../../data/ArcTooSmall.nc",
      "../../data/TortureTestLines.nc",
      "../../data/TortureTestSmallFilletsG91.nc",
      "../../data/SimpleSquarePocket.nc",
      "../../data/SimpleSquarePocketOverlap.nc",
      "../../data/CompErrorTest.nc",
      "../../data/PauseMarkers.nc",
      "../../data/MultipleZmoves.nc",
      "../../data/CompErrorTest.nc",
      "../../data/Comp_Err_out_Test.nc"};

  if (argc > 1 && std::strcmp(argv[1], "--sweep-self-test") == 0)
    return sweep_self_test();
  if (argc > 1 && std::strcmp(argv[1], "--sweep") == 0)
    return sweep_main(argc, argv, default_files, sizeof(default_files) / sizeof(default_files[0]));

  std::vector<const char *> inputFiles;
  if (argc > 1 && std::strcmp(argv[1], "all") != 0)
    inputFiles.push_back(argv[1]);
  else
    inputFiles.assign(std::begin(default_files), std::end(default_files));

  std::string outputFolder = (argc > 2) ? argv[2] : "../../output/";
  if (!outputFolder.empty() && outputFolder.back() != '/' && outputFolder.back() != '\\')
    outputFolder += '/';

  float configuredToolRadius = TOOL_RADIUS;
  if (argc > 3)
  {
    try
    {
      configuredToolRadius = std::stof(argv[3]);
    }
    catch (...)
    {
      std::fprintf(stderr, "Invalid tool radius: %s\n", argv[3]);
      configuredToolRadius = TOOL_RADIUS;
    }
  }

  CornerType cornerTreatment = CORNER_TREATMENT;
  if (argc > 4)
  {
    std::string ctArg = argv[4];
    if (ctArg == "roll" || ctArg == "ROLL")
      cornerTreatment = CORNER_ROLL;
    else if (ctArg == "chamfer" || ctArg == "CHAMFER")
      cornerTreatment = CORNER_CHAMFER;
    else
      std::fprintf(stderr, "Unknown corner treatment: %s (using default)\n", argv[4]);
  }

  bool outputSVG = OUTPUT_SVG;
  if (argc > 5)
  {
    std::string svgArg = argv[5];
    if (svgArg == "nosvg" || svgArg == "NOSVG")
      outputSVG = false;
    else if (svgArg == "svg" || svgArg == "SVG")
      outputSVG = true;
    else
      std::fprintf(stderr, "Unknown SVG option: %s (using default)\n", argv[5]);
  }

  int result = 0;
  for (const char *input_file : inputFiles)
  {
    const std::string inputFilePath(input_file);
    g_currentInputFile = inputFilePath;
    const std::string inputBaseName = basename_no_ext(inputFilePath);
    std::printf("Processing: %s\n", input_file);

    // Get input file extension (if any)
    std::string inputExt;
    size_t dotPos = inputFilePath.find_last_of('.');
    if (dotPos != std::string::npos && dotPos > inputFilePath.find_last_of("/\\"))
    {
      inputExt = inputFilePath.substr(dotPos);
    }
    else
    {
      inputExt = ".ngc";
    }

    const std::string outBaseName = inputBaseName;
    const std::string svgPath = outputFolder + outBaseName + ".host.svg";
    const std::string simpleSvgPath = outputFolder + outBaseName + ".mcu.svg";
    const std::string comparePath = outputFolder + outBaseName + ".compare.txt";
    const std::string ngcPath = outputFolder + outBaseName + inputExt;

    std::vector<std::string> program = load_program_from_file(input_file);
    if (program.empty())
    {
      result = 1;
      continue;
    }

    float toolRadius = configuredToolRadius;
    if (toolRadius == 0.0f)
      toolRadius = comp_tool_radius_from_diameter(program);

    const bool toolInches = comp_starts_in_inches(program);
    const float mcuRadiusMm = toolInches ? toolRadius * 25.4f : toolRadius;

    const bool isvalid = run_profile_streaming(input_file, ngcPath.c_str(), toolRadius, cornerTreatment);
    if (!isvalid)
    {
      result = 1;
      std::fprintf(stderr, "%s: Host profile validation failed; output is incomplete\n", input_file);
    }

    std::vector<Move2D> simpleCompensated;
    const bool simpleValid = run_profile_simple_xy(program, toolRadius, cornerTreatment, simpleCompensated);
    if (!simpleValid)
    {
      result = 1;
      std::fprintf(stderr, "%s: MCU simulation failed; output is incomplete\n", input_file);
    }

    std::vector<std::string> compProgram = load_program_from_file(ngcPath.c_str());
    auto compensated = build_original_moves(compProgram);
    auto compensatedMm = build_original_moves(compProgram, true);
    std::vector<Move2D> compensatedVisible = filter_valid_moves(compensated);

    if (outputSVG)
    {
      auto orig = build_original_moves(program);
      const size_t invalidFullCount = count_invalid_moves(compensated);
      const size_t invalidSimpleCount = count_invalid_moves(simpleCompensated);
      std::vector<Move2D> simpleVisible = filter_valid_moves(simpleCompensated);
      std::vector<Move2D> simpleCompareVisible = filter_compare_moves(simpleCompensated);
      std::vector<Move2D> compensatedCompareVisible = filter_compare_moves(compensatedMm, compProgram);
      write_xy_compare_report(comparePath.c_str(), compensatedCompareVisible, simpleCompareVisible, compProgram, program,
                              invalidFullCount, invalidSimpleCount);

      write_svg(svgPath.c_str(), compensatedVisible, &orig, false, true, fabs(toolRadius * 2.0f),
                false, true, false, inputBaseName.c_str(), toolRadius, false, "Host", false,
                toolInches ? UNITS_INCH : UNITS_MM); // mirror for better visualization
      auto originalMm = build_original_moves(program, true);
      write_svg(simpleSvgPath.c_str(), simpleVisible, &originalMm, false, true, fabs(mcuRadiusMm * 2.0f),
                false, true, false, inputBaseName.c_str(), mcuRadiusMm, false, "MCU", true, UNITS_MM);
      //std::printf("Wrote: %s, %s, %s, %s (full)\n", svgPath.c_str(), simpleSvgPath.c_str(), comparePath.c_str(), ngcPath.c_str());
    }
    else
    {
      const size_t invalidFullCount = count_invalid_moves(compensated);
      const size_t invalidSimpleCount = count_invalid_moves(simpleCompensated);
      std::vector<Move2D> simpleVisible = filter_compare_moves(simpleCompensated);
      std::vector<Move2D> compensatedCompareVisible = filter_compare_moves(compensatedMm, compProgram);
      write_xy_compare_report(comparePath.c_str(), compensatedCompareVisible, simpleVisible, compProgram, program,
                              invalidFullCount, invalidSimpleCount);
      std::printf("Wrote: %s (full)\n", ngcPath.c_str());
      std::printf("Standalone cc_xy moves: %zu\n", simpleVisible.size());
      std::printf("Compare report: %s\n", comparePath.c_str());
    }
  }
  return result;
}
