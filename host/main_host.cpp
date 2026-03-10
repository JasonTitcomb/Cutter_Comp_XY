#include <cstdio>
#include <vector>
#include <string>
#include <cstring>
#include <fstream>
#include <sstream>
#include <cctype>
#include <iostream>

#define DBG_PRINTLN(x)             \
  do                               \
  {                                \
    std::cout << (x) << std::endl; \
  } while (0)
#define DBG_PRINT(x, ...) std::printf(x, ##__VA_ARGS__)

#include "cc_simple_scan.h"
#include "cc_main.h"
#include "writer.h"
/*
  This is a desktop test harness for the CutterComp2D class, which performs 2D cutter compensation on linear and arc moves.

  It includes a post-pass to trim crossing elements, which is a common source of tiny unwanted moves after compensation. This is optional and can be toggled with ENABLE_TRIM_CROSSINGS.

  The test uses hardcoded G-code in test_data.h, which you can modify to test different scenarios. The output is printed as G-code lines, and also saved to "output.csv" for analysis and "output.svg" for visualization.

  Note: This code is meant for testing the compensation logic on the host. It does not run on an Arduino or control any hardware.
*/

// -------------------- Config --------------------
static constexpr bool STOP_ON_FIRST_ERRORS = true;
static constexpr float TOOL_RADIUS = 0.005f;
static constexpr CornerType CORNER_TREATMENT = CORNER_ROLL; // CORNER_ROLL or CORNER_CHAMFER
static constexpr bool TRIM_CROSSING = true;
static constexpr bool MERGE_COLINEAR = true;
static constexpr int MAX_LOOKAHEAD = 10;

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
  if (err != CE_NONE)
    std::fprintf(stderr, "CompError code=%u N%u\n", (unsigned)err, (unsigned)seqNum);
}

// -------------------- Profile buffer --------------------
static std::vector<Move2D> profile;
static int profileCount = 0;

static inline void copy_gcode_line(char *dst, size_t dstSize, const char *src)
{
#ifdef _MSC_VER
  strncpy_s(dst, dstSize, src, _TRUNCATE);
#else
  std::strncpy(dst, src, dstSize - 1);
  dst[dstSize - 1] = '\0';
#endif
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

  FILE *out = open_file_write_binary(emitGcodePath);
  if (!out)
  {
    std::fprintf(stderr, "Failed to open output file: %s\n", emitGcodePath);
    return false;
  }

  HostRunnerContext ctx;

  ctx.out = out;

  g_hostRunnerContext = &ctx;

  CcMainRunner runner;
  CcMainOptions options;
  options.toolRadius = toolRadius;
  options.cornerTreatment = cornerTreatment;
  options.trimCrossing = TRIM_CROSSING;
  options.outputInchUnits = true;
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

  std::fclose(out);
  return ok && !ctx.sawError;
}

int main()
{
  //  const char *default_file = "../../data/RapidComp.nc";
  //  const char *default_file = "../../data/G41_1.nc";
  //  const char *default_file = "../../data/G41_2.nc";
  //  const char *default_file = "../../data/TortureTestG91.nc";
  //  const char *default_file = "../../data/Sample2.nc";
  //  const char *default_file = "../../data/ArcExtension_Test_ArcArc_1.nc";
  //  const char *default_file = "../../data/TortureTestmm.nc";
  //  const char *default_file = "../../data/simple1.nc";
  const char *default_file = "../../data/TortureTestG90.nc";
  //const char *default_file = "../../data/TortureTestLines.nc";
  // const char *default_file = "../../data/AI_Torture.nc";
  // const char *default_file = "../../data/TortureTestSmallFilletsG91.nc";
  //  const char *default_file = "../../data/SimpleSquarePocket.nc";
  // const char *default_file = "../../data/SimpleSquarePocketOverlap.nc";
  // const char *default_file = "../../data/CompErrorTest.nc";
  // const char *default_file = "../../data/Tangent_ArcLine.nc";
  const std::string inputFilePath(default_file);
  const std::string inputBaseName = basename_no_ext(inputFilePath);

  float toolRadius = TOOL_RADIUS;

  CornerType cornerTreatment = CORNER_TREATMENT;
  const std::string outBaseName = std::string(inputBaseName);
  const std::string svgPath = "../../output/" + outBaseName + ".svg";
  const std::string ngcPath = "../../output/" + outBaseName + ".ngc";

  const bool isvalid = run_profile_streaming(default_file, ngcPath.c_str(), toolRadius, cornerTreatment);
  if (!isvalid)
    std::puts("(warning: profile validation failed)");

  std::vector<std::string> program = load_program_from_file(default_file);
  if (program.empty())
    return 1;

  auto orig = build_original_moves(program);
  std::vector<std::string> compProgram = load_program_from_file(ngcPath.c_str());
  auto compensated = build_original_moves(compProgram);

  write_svg(svgPath.c_str(), compensated, &orig, false, true, fabs(toolRadius * 2.0f),
            false, true, false, inputBaseName.c_str(), toolRadius); // mirror for better visualization

  std::printf("Wrote: %s, %s\n", svgPath.c_str(), ngcPath.c_str());
  return 0;
}
