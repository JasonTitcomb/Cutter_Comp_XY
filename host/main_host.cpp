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

static constexpr float TOOL_RADIUS = 0.0620f;
static constexpr CornerType CORNER_TREATMENT = CORNER_ROLL; // CORNER_ROLL or CORNER_CHAMFER
static constexpr bool GLOBAL_TRIM_CROSSING = true;

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

static void start_comp_cb(int toolRegister, int diaRegister)
{
  // placeholder for start of comp callback, which could be used to log or track when compensation starts, and with which tool/dia registers.
}

static void host_error_cb(const char *message, CompError err, uint32_t seqNum)
{
  if (message)
    std::fprintf(stderr, "%s\n", message);
  if (err != CE_ERROR)
    std::fprintf(stderr, "CompError code=%u N%u\n", (unsigned)err, (unsigned)seqNum);
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
  options.callbacks.output = host_output_cb;
  options.callbacks.error = host_error_cb;
  options.callbacks.startComp = start_comp_cb;

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
  // const char *default_file = "../../data/G41_2.nc";
  //  const char *default_file = "../../data/TortureTestG91.nc";
  //  const char *default_file = "../../data/Sample2.nc";
  //  const char *default_file = "../../data/ArcExtension_Test_ArcArc_1.nc";
  //  const char *default_file = "../../data/TortureTestmm.nc";
  //  const char *default_file = "../../data/simple1.nc";
  const char *default_file = "../../data/TortureTestG90.nc";
  // const char *default_file = "../../data/TortureTestLinux.nc";
  // const char *default_file = "../../data/ArcTooSmall.nc";
  // const char *default_file = "../../data/TortureTestLines.nc";
  // const char *default_file = "../../data/AI_Torture.nc";
  // const char *default_file = "../../data/TortureTestSmallFilletsG91.nc";
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

  bool outputSVG = false;
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
  const std::string ngcPath = outputFolder + outBaseName + inputExt;

  const bool isvalid = run_profile_streaming(input_file, ngcPath.c_str(), toolRadius, cornerTreatment);
  if (!isvalid)
    std::puts("(warning: profile validation failed)");

  std::vector<std::string> program = load_program_from_file(input_file);
  if (program.empty())
    return 1;

  if (outputSVG)
  {
    auto orig = build_original_moves(program);
    std::vector<std::string> compProgram = load_program_from_file(ngcPath.c_str());
    auto compensated = build_original_moves(compProgram);

    write_svg(svgPath.c_str(), compensated, &orig, false, true, fabs(toolRadius * 2.0f),
              false, true, false, inputBaseName.c_str(), toolRadius); // mirror for better visualization
    std::printf("Wrote: %s, %s\n", svgPath.c_str(), ngcPath.c_str());
  }
  else
  {
    std::printf("Wrote: %s\n", ngcPath.c_str());
  }
  return 0;
}
