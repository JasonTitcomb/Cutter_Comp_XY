#include <cstdio>
#include <vector>
#include <string>
#include <cstring>
#include <fstream>
#include <sstream>
#include <cctype>
#include <iostream>
#include <iomanip>

#define DBG_PRINTLN(x)             \
  do                               \
  {                                \
    std::cout << (x) << std::endl; \
  } while (0)
#define DBG_PRINT(x, ...) std::printf(x, ##__VA_ARGS__)

#include "cc_simple_scan.h"
#include "cc_main.h"
#include "cc_xy_adapter.h"
#include "cc_xy_host_bridge.h"
#include "writer.h"
/*
  This is a desktop test harness for the CutterComp2D class, which performs 2D cutter compensation on linear and arc moves.

  It includes a post-pass to trim crossing elements, which is a common source of tiny unwanted moves after compensation. This is optional and can be toggled with ENABLE_TRIM_CROSSINGS.

  The test uses hardcoded G-code in test_data.h, which you can modify to test different scenarios. The output is printed as G-code lines, and also saved to "output.csv" for analysis and "output.svg" for visualization.

  Note: This code is meant for testing the compensation logic on the host. It does not run on an Arduino or control any hardware.
*/

// -------------------- Config --------------------

static constexpr float TOOL_RADIUS = 0.0625f;
static constexpr CornerType CORNER_TREATMENT = CORNER_ROLL; // CORNER_ROLL or CORNER_CHAMFER
static constexpr bool GLOBAL_TRIM_CROSSING = true;
static constexpr bool GLOBAL_MERGE = true;
static constexpr bool OUTPUT_SVG = true;

// ------------------------------------------------
static CutterComp2D cc;
struct HostRunnerContext
{
  FILE *out = nullptr;
  bool sawError = false;
};

static HostRunnerContext *g_hostRunnerContext = nullptr;

static void host_output_cb(const char *text, size_t len)
{
  if (!g_hostRunnerContext || !g_hostRunnerContext->out || !text || len == 0)
    return;
  fwrite(text, 1, len, g_hostRunnerContext->out);
}


static void host_error_cb(const char *message, CompError err, uint32_t seqNum)
{
  if (message)
    std::fprintf(stderr, "%s\n", message);
  if (err != CE_ERROR)
    std::fprintf(stderr, "CompError code=%u N%u\n", (unsigned)err, (unsigned)seqNum);
}

static void host_xy_error_cb(const char *message, cc_comp_status err, uint32_t seqNum)
{
  if (message)
    std::fprintf(stderr, "[cc_xy] %s\n", message);
  if (err != CC_ERROR)
    std::fprintf(stderr, "[cc_xy] CompError code=%u N%u\n", (unsigned)err, (unsigned)seqNum);
}

static std::vector<Move2D> *g_simpleProfileOut = nullptr;
static float g_mc_host_pos[N_AXIS] = {};

extern "C" {

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
    mv.type = (pl_data && pl_data->condition.rapid_motion) ? MOT_RAPID : MOT_LINE;
    mv.valid = true;
    g_simpleProfileOut->push_back(mv);
  }
  g_mc_host_pos[0] = xyz[0];
  g_mc_host_pos[1] = xyz[1];
  g_mc_host_pos[2] = xyz[2];
  return true;
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

static std::vector<Move2D> build_original_moves(const std::vector<std::string> &program)
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

  for (const auto &line : program)
  {
    char clean[160];
    strip_comments(line.c_str(), clean, sizeof(clean));
    ScanLine s;
    scan_line(clean, s);
    Move2D mv = interpret_move(s, m);
    if (mv.type != MOT_EMPTY)
    {
      mv.valid = true;
      orig.push_back(mv);
    }
  }
  return orig;
}

static bool run_profile_simple_xy(const std::vector<std::string> &program,
                                  float toolRadius,
                                  std::vector<Move2D> &profileOut)
{
  ModalState modal{};
  modal.planeXY = true;
  modal.absoluteMode = true;
  modal.motionG = 0;
  modal.comp = COMP_OFF;
  modal.feed = 0;
  modal.pos = v2(0, 0);
  modal.z = 0.0f;

  g_mc_host_pos[0] = 0.0f;
  g_mc_host_pos[1] = 0.0f;
  g_mc_host_pos[2] = 0.0f;

  cc_api_init(toolRadius,cc_emit_via_mc, host_xy_error_cb);

  g_simpleProfileOut = &profileOut;

  for (const auto &line : program)
  {
    char clean[160];
    strip_comments(line.c_str(), clean, sizeof(clean));

    ScanLine s;
    scan_line(clean, s);

    Move2D mv = interpret_move(s, modal);

    if (s.sawG41 || s.sawG42)
      cc_api_set_comp(cc_from_comp_side(modal.comp));

    if (mv.type == MOT_EMPTY)
    {
      if (s.sawG40)
      {
        if (cc_api_process_move(nullptr) != CC_OK)
        {
          g_simpleProfileOut = nullptr;
          return false;
        }
        cc_api_set_comp(CC_COMP_OFF);
      }
      continue;
    }

    move2d xyMove = cc_from_move2d(mv);
    if (cc_api_process_move(&xyMove) != CC_OK)
    {
      g_simpleProfileOut = nullptr;
      return false;
    }

    if (s.sawG40)
    {
      if (cc_api_process_move(nullptr) != CC_OK)
      {
        g_simpleProfileOut = nullptr;
        return false;
      }
      cc_api_set_comp(CC_COMP_OFF);
    }
  }

  if (cc_api_process_move(nullptr) != CC_OK)
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

static void write_xy_compare_report(const char *path,
                                    const std::vector<Move2D> &fullProfile,
                                    const std::vector<Move2D> &simpleProfile)
{
  std::ofstream out(path);
  const float compareTol = 0.001f;
  const size_t sharedCount = std::min(fullProfile.size(), simpleProfile.size());
  size_t mismatchCount = 0;
  size_t typeMismatchCount = 0;
  size_t invalidMismatchCount = 0;
  int firstMismatch = -1;
  float maxP0Delta = 0.0f;
  float maxP1Delta = 0.0f;
  float maxCenterDelta = 0.0f;
  float maxRadiusDelta = 0.0f;
  if (!out)
    return;

  out << "cc_xy comparison report\n";
  out << "full profile count: " << fullProfile.size() << "\n";
  out << "simple profile count: " << simpleProfile.size() << "\n";
  out << "shared count: " << sharedCount << "\n";

  for (size_t i = 0; i < sharedCount; ++i)
  {
    const Move2D &full = fullProfile[i];
    const Move2D &simple = simpleProfile[i];
    const float p0Delta = compare_vec_delta(full.p_0, simple.p_0);
    const float p1Delta = compare_vec_delta(full.p_1, simple.p_1);
    const float centerDelta = compare_vec_delta(full.center, simple.center);
    const float radiusDelta = fabsf(full.radius - simple.radius);
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

    if ((full.type == MOT_ARC || simple.type == MOT_ARC) &&
        (centerDelta > compareTol || radiusDelta > compareTol))
      mismatch = true;

    if (!mismatch)
      continue;

    if (firstMismatch < 0)
      firstMismatch = (int)i;

    ++mismatchCount;
    if (mismatchCount <= 20)
    {
      out << "\nindex " << i << " mismatch\n";
      out << "  full   : type=" << motion_type_name(full.type)
          << " seq=" << full.seqNum
          << " p0=(" << full.p_0.x << ", " << full.p_0.y << ")"
          << " p1=(" << full.p_1.x << ", " << full.p_1.y << ")"
          << " r=" << full.radius << " valid=" << full.valid << "\n";
      out << "  simple : type=" << motion_type_name(simple.type)
          << " seq=" << simple.seqNum
          << " p0=(" << simple.p_0.x << ", " << simple.p_0.y << ")"
          << " p1=(" << simple.p_1.x << ", " << simple.p_1.y << ")"
          << " r=" << simple.radius << " valid=" << simple.valid << "\n";
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

// Command line usage:
// main_host [inputfile] [outputfolder] [toolradius] [cornerTreatment] [svg]
// Defaults:
//   inputfile: default_file
//   outputfolder: "../../output/"
//   toolradius: TOOL_RADIUS
//   cornerTreatment: CORNER_TREATMENT ("roll" or "chamfer")
//   svg: "svg" (default, output SVG), or "nosvg" (do not output SVG)
int main(int argc, char *argv[])
{
  //  const char *default_file = "../../data/RapidComp.nc";
  //  const char *default_file = "../../data/G41_1.nc";
  // const char *default_file = "../../data/ThreadMill.nc";
  //const char *default_file = "../../data/G41_2.nc";
  //  const char *default_file = "../../data/TortureTestG91.nc";
  //  const char *default_file = "../../data/Sample2.nc";
  //  const char *default_file = "../../data/ArcExtension_Test_ArcArc_1.nc";
  //  const char *default_file = "../../data/TortureTestmm.nc";
  //  const char *default_file = "../../data/simple1.nc";
  //const char *default_file = "../../data/TortureTestG90.nc";
  // const char *default_file = "../../data/TortureTestLinux.nc";
  // const char *default_file = "../../data/ArcTooSmall.nc";
  // const char *default_file = "../../data/TortureTestLines.nc";
  // const char *default_file = "../../data/AI_Torture.nc";
   const char *default_file = "../../data/TortureTestSmallFilletsG91.nc";
  //  const char *default_file = "../../data/SimpleSquarePocket.nc";
  // const char *default_file = "../../data/SimpleSquarePocketOverlap.nc";
  // const char *default_file = "../../data/CompErrorTest.nc";
  // const char *default_file = "../../data/Tangent_ArcLine.nc";

  const char *input_file = (argc > 1) ? argv[1] : default_file;
  const std::string inputFilePath(input_file);
  const std::string inputBaseName = basename_no_ext(inputFilePath);

  std::string outputFolder = (argc > 2) ? argv[2] : "../../output/";
  if (!outputFolder.empty() && outputFolder.back() != '/' && outputFolder.back() != '\\')
    outputFolder += '/';

  float toolRadius = TOOL_RADIUS;
  if (argc > 3)
  {
    try
    {
      toolRadius = std::stof(argv[3]);
    }
    catch (...)
    {
      std::fprintf(stderr, "Invalid tool radius: %s\n", argv[3]);
      toolRadius = TOOL_RADIUS;
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
  const std::string svgPath = outputFolder + outBaseName + ".svg";
  const std::string simpleSvgPath = outputFolder + outBaseName + ".xy.svg";
  const std::string comparePath = outputFolder + outBaseName + ".xy.compare.txt";
  const std::string ngcPath = outputFolder + outBaseName + inputExt;

  const bool isvalid = run_profile_streaming(input_file, ngcPath.c_str(), toolRadius, cornerTreatment);
  if (!isvalid)
    std::puts("(warning: profile validation failed)");

  std::vector<std::string> program = load_program_from_file(input_file);
  if (program.empty())
    return 1;

  std::vector<Move2D> simpleCompensated;
  const bool simpleValid = run_profile_simple_xy(program, toolRadius, simpleCompensated);
  if (!simpleValid)
    std::puts("(warning: standalone cc_xy validation failed)");

  if (outputSVG)
  {
    auto orig = build_original_moves(program);
    std::vector<std::string> compProgram = load_program_from_file(ngcPath.c_str());
    auto compensated = build_original_moves(compProgram);
    write_xy_compare_report(comparePath.c_str(), compensated, simpleCompensated);

    write_svg(svgPath.c_str(), compensated, &orig, false, true, fabs(toolRadius * 2.0f),
              false, true, false, inputBaseName.c_str(), toolRadius); // mirror for better visualization
    write_svg(simpleSvgPath.c_str(), simpleCompensated, &orig, false, true, fabs(toolRadius * 2.0f),
              false, true, false, inputBaseName.c_str(), toolRadius);
    std::printf("Wrote: %s, %s, %s, %s\n", svgPath.c_str(), simpleSvgPath.c_str(), comparePath.c_str(), ngcPath.c_str());
  }
  else
  {
    std::vector<std::string> compProgram = load_program_from_file(ngcPath.c_str());
    auto compensated = build_original_moves(compProgram);
    write_xy_compare_report(comparePath.c_str(), compensated, simpleCompensated);
    std::printf("Wrote: %s\n", ngcPath.c_str());
    std::printf("Standalone cc_xy moves: %zu\n", simpleCompensated.size());
    std::printf("Compare report: %s\n", comparePath.c_str());
  }
  return 0;
}
