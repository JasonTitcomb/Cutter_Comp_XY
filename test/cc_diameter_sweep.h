#pragma once

#include <algorithm>
#include <cmath>
#include <system_error>

static constexpr double SWEEP_START_IN = 0.0001;
static constexpr double SWEEP_MAX_IN = 1.0;
static constexpr double SWEEP_FACTOR = 1.01123;

struct SweepAttempt
{
  double diameterIn = 0.0;
  bool success = false;
  SweepDiagnostic diagnostic;
};

static double sweep_next_diameter(double diameter)
{
  return std::min(SWEEP_MAX_IN, diameter * SWEEP_FACTOR);
}

static size_t g_sweepOutputBytes = 0;
static std::string *g_sweepCapturedGcode = nullptr;

static void sweep_output_cb(const char *text, size_t length)
{
  if (text)
  {
    g_sweepOutputBytes += length;
    if (g_sweepCapturedGcode)
      g_sweepCapturedGcode->append(text, length);
  }
}

static SweepAttempt sweep_attempt(const std::vector<std::string> &program,
                                  double diameterIn, bool mcu,
                                  std::vector<Move2D> *outputMoves = nullptr)
{
  SweepAttempt attempt;
  attempt.diameterIn = diameterIn;
  g_sweepDiagnostic = &attempt.diagnostic;
  const float radiusMm = (float)(diameterIn * 25.4 * 0.5);
  if (mcu)
  {
    std::vector<Move2D> moves;
    attempt.success = run_profile_simple_xy(program, radiusMm, CORNER_TREATMENT, moves, true);
    if (outputMoves)
      *outputMoves = moves;
    if (attempt.success && moves.empty())
    {
      attempt.success = false;
      attempt.diagnostic.message = "MCU emitted no moves";
    }
  }
  else
  {
    CcMainRunner runner;
    std::string capturedGcode;
    g_sweepCapturedGcode = outputMoves ? &capturedGcode : nullptr;
    CcMainOptions options;
    options.toolRadius = radiusMm;
    options.cornerTreatment = CORNER_TREATMENT;
    options.globalTrimCrossing = GLOBAL_TRIM_CROSSING;
    options.globalMerge = GLOBAL_MERGE;
    options.emitStatusComments = false;
    options.callbacks.output = sweep_output_cb;
    options.callbacks.error = host_error_cb;
    g_sweepOutputBytes = 0;
    attempt.success = runner.begin(options);
    bool inchMode = false;
    for (const std::string &line : program)
    {
      if (!attempt.success || attempt.diagnostic.sawError)
        break;
      char clean[160];
      strip_comments(line.c_str(), clean, sizeof(clean));
      ScanLine scan;
      scan_line(clean, scan);
      if (scan.sawG20)
        inchMode = true;
      if (scan.sawG21)
        inchMode = false;
      runner.setToolRadius(inchMode ? radiusMm / 25.4f : radiusMm);
      runner.incrementLineNumber();
      attempt.success = runner.processLine(line.c_str());
    }
    if (attempt.success && !attempt.diagnostic.sawError)
      attempt.success = runner.finish();
    if (attempt.success && g_sweepOutputBytes == 0)
    {
      attempt.success = false;
      attempt.diagnostic.message = "Host emitted no output";
    }
    g_sweepCapturedGcode = nullptr;
    if (outputMoves)
    {
      std::vector<std::string> outputLines;
      std::istringstream stream(capturedGcode);
      std::string line;
      while (std::getline(stream, line))
        outputLines.push_back(line);
      *outputMoves = build_original_moves(outputLines, true);
    }
  }
  g_sweepDiagnostic = nullptr;
  attempt.success = attempt.success && !attempt.diagnostic.sawError;
  if (!attempt.success && attempt.diagnostic.message.empty())
    attempt.diagnostic.message = "Processing failed without an error callback";
  return attempt;
}

static std::vector<SweepAttempt> sweep_program(const std::vector<std::string> &program, bool mcu)
{
  std::vector<SweepAttempt> attempts;
  double diameterIn = SWEEP_START_IN;
  for (;;)
  {
    attempts.push_back(sweep_attempt(program, diameterIn, mcu));
    if (!attempts.back().success || diameterIn == SWEEP_MAX_IN)
      break;
    diameterIn = sweep_next_diameter(diameterIn);
  }
  return attempts;
}

static std::string sweep_html_escape(const std::string &text)
{
  std::string escaped;
  for (char character : text)
  {
    switch (character)
    {
    case '&': escaped += "&amp;"; break;
    case '<': escaped += "&lt;"; break;
    case '>': escaped += "&gt;"; break;
    case '"': escaped += "&quot;"; break;
    default: escaped += character; break;
    }
  }
  return escaped;
}

struct SweepFileResult
{
  std::string input;
  bool inches = false;
  std::vector<SweepAttempt> attempts;
};

static double sweep_final_diameter(const std::vector<SweepAttempt> &attempts)
{
  double diameter = 0.0;
  for (const SweepAttempt &attempt : attempts)
    if (attempt.success)
      diameter = attempt.diameterIn;
  return diameter;
}

static size_t sweep_increment_count(const std::vector<SweepAttempt> &attempts)
{
  return attempts.empty() ? 0 : attempts.size() - 1;
}

static void sweep_write_file_details(std::ostream &report, const SweepFileResult &file)
{
  const std::vector<SweepAttempt> &attempts = file.attempts;
  const double finalDiameter = sweep_final_diameter(attempts);
  const SweepAttempt &last = attempts.back();
  report << "<details><summary>" << sweep_html_escape(file.input) << "</summary>"
         << "<p>Units at first compensation entry: " << (file.inches ? "inch" : "mm") << ".</p>"
         << "<p><strong>Final successfully processed tool diameter: ";
  if (finalDiameter > 0.0)
    report << finalDiameter << " in / " << finalDiameter * 25.4 << " mm";
  else
    report << "None (the initial diameter failed)";
  report << "</strong></p><p>Termination: "
         << (last.success ? "Reached maximum diameter (1.0 in)." : "Stopped at first processing error.")
         << " Attempts: " << attempts.size()
         << "; increments executed: " << sweep_increment_count(attempts) << ".</p>";
  if (!last.success)
    report << "<p>First failed diameter: " << last.diameterIn << " in / "
           << last.diameterIn * 25.4 << " mm; error code: " << last.diagnostic.code
           << "; source line: " << last.diagnostic.line << "; "
           << sweep_html_escape(last.diagnostic.message) << "</p>";
  report << "<table><thead><tr><th>Attempt</th><th>Diameter (in)</th><th>Diameter (mm)</th>"
         << "<th>Offset radius (entry units)</th><th>Result</th><th>Error code</th>"
         << "<th>Source line</th><th>Message</th></tr></thead><tbody>";
  for (size_t index = 0; index < attempts.size(); ++index)
  {
    const SweepAttempt &attempt = attempts[index];
    report << "<tr><td>" << index + 1 << "</td><td>" << attempt.diameterIn << "</td><td>"
           << attempt.diameterIn * 25.4 << "</td><td>"
           << attempt.diameterIn * (file.inches ? 1.0 : 25.4) * 0.5 << "</td><td>"
           << (attempt.success ? "Success" : "Error") << "</td><td>"
           << attempt.diagnostic.code << "</td><td>" << attempt.diagnostic.line << "</td><td>"
           << sweep_html_escape(attempt.diagnostic.message) << "</td></tr>";
  }
  report << "</tbody></table></details>\n";
}

static bool sweep_write_report(const std::filesystem::path &path, bool mcu,
                               const std::vector<SweepFileResult> &files)
{
  std::ofstream report(path, std::ios::trunc);
  if (!report)
  {
    std::fprintf(stderr, "Cannot open sweep report: %s\n", path.string().c_str());
    return false;
  }
  report << "<!doctype html><html lang=\"en\"><head><meta charset=\"utf-8\">"
         << "<title>" << (mcu ? "MCU" : "Host") << " diameter sweep</title>"
         << "<style>body{font-family:system-ui;margin:2em}table{border-collapse:collapse}"
         << "th,td{border:1px solid #aaa;padding:.4em;text-align:left}"
         << "details{margin:1em 0}summary{cursor:pointer;font-weight:bold}</style></head><body>"
         << "<h1>" << (mcu ? "MCU" : "Host") << " diameter sweep</h1><p>Engine: "
         << (mcu ? "MCU C99 core / desktop grblHAL shim" : "Host C++ runner")
         << "</p><p>Tool diameter: 0.0001 in to 1.0 in; increment: 1.123% "
         << "(multiply by 1.01123); offset radius = diameter / 2. "
         << "Roll corners, crossing lookahead enabled, merging disabled.</p>"
         << "<p>Increments executed counts diameter increases after the initial 0.0001 in attempt, "
         << "including the increase to a failed attempt or the clamped 1.0 in endpoint. "
         << "Total attempts = increments + 1.</p>"
         << "<p>Files processed: " << files.size() << ".</p>"
         << "<table><thead><tr><th>File</th><th>Entry units</th><th>Final diameter (in)</th>"
         << "<th>Final diameter (mm)</th><th>Increments executed</th><th>Total attempts</th>"
         << "<th>Termination</th><th>First failed diameter (in)</th><th>Error code</th>"
         << "<th>Source line</th><th>Message</th></tr></thead><tbody>"
         << std::fixed << std::setprecision(9);
  for (const SweepFileResult &file : files)
  {
    const double finalDiameter = sweep_final_diameter(file.attempts);
    const SweepAttempt &last = file.attempts.back();
    report << "<tr><td>" << sweep_html_escape(std::filesystem::path(file.input).filename().string())
           << "</td><td>" << (file.inches ? "inch" : "mm") << "</td><td>";
    if (finalDiameter > 0.0)
      report << finalDiameter << "</td><td>" << finalDiameter * 25.4;
    else
      report << "None</td><td>None";
    report << "</td><td>" << sweep_increment_count(file.attempts) << "</td><td>"
           << file.attempts.size() << "</td><td>" << (last.success ? "Maximum reached" : "First error")
           << "</td><td>";
    if (!last.success)
      report << last.diameterIn;
    else
      report << "-";
    report << "</td><td>" << last.diagnostic.code << "</td><td>" << last.diagnostic.line
           << "</td><td>" << sweep_html_escape(last.diagnostic.message) << "</td></tr>";
  }
  report << "</tbody></table><h2>Per-file attempt details</h2>";
  for (const SweepFileResult &file : files)
    sweep_write_file_details(report, file);
  report << "</body></html>\n";
  report.close();
  if (!report)
  {
    std::fprintf(stderr, "Failed writing sweep report: %s\n", path.string().c_str());
    return false;
  }
  return true;
}

static int sweep_main(int argc, char *argv[], const char *const *defaultFiles, size_t fileCount)
{
  if (argc < 3 || argc > 5 ||
      (std::strcmp(argv[2], "host") != 0 && std::strcmp(argv[2], "mcu") != 0))
  {
    std::fprintf(stderr, "Usage: cc_runner --sweep host|mcu [inputfile|all] [reportfolder]\n");
    return 1;
  }
  const bool mcu = std::strcmp(argv[2], "mcu") == 0;
  const std::filesystem::path output = argc > 4 ? std::filesystem::path(argv[4]) :
      std::filesystem::path(CC_SWEEP_DATA_DIR).parent_path() / "output" / "diameter-sweep";
  std::error_code error;
  std::filesystem::create_directories(output, error);
  if (error)
  {
    std::fprintf(stderr, "Cannot create report directory: %s\n", error.message().c_str());
    return 1;
  }
  std::vector<std::string> inputs;
  if (argc > 3 && std::strcmp(argv[3], "all") != 0)
    inputs.push_back(argv[3]);
  else
    for (size_t index = 0; index < fileCount; ++index)
    {
      const std::string input = (std::filesystem::path(CC_SWEEP_DATA_DIR) /
                                std::filesystem::path(defaultFiles[index]).filename()).string();
      if (std::find(inputs.begin(), inputs.end(), input) == inputs.end())
        inputs.push_back(input);
    }
  int result = 0;
  std::vector<SweepFileResult> files;
  for (const std::string &input : inputs)
  {
    const std::vector<std::string> program = load_program_from_file(input.c_str());
    if (program.empty())
    {
      std::fprintf(stderr, "Missing or empty sweep input: %s\n", input.c_str());
      result = 1;
      continue;
    }
    bool sawComp = false;
    for (const std::string &line : program)
    {
      char clean[160];
      strip_comments(line.c_str(), clean, sizeof(clean));
      ScanLine scan;
      scan_line(clean, scan);
      sawComp = sawComp || scan.sawG41 || scan.sawG42;
    }
    if (!sawComp)
    {
      std::fprintf(stderr, "Sweep input has no G41/G42 profile: %s\n", input.c_str());
      result = 1;
      continue;
    }
    g_currentInputFile = input;
    const std::vector<SweepAttempt> attempts = sweep_program(program, mcu);
    files.push_back({input, comp_starts_in_inches(program), attempts});
    const double finalDiameter = attempts.back().success ? attempts.back().diameterIn :
                                 (attempts.size() > 1 ? attempts[attempts.size() - 2].diameterIn : 0.0);
    std::printf("%s %s: final diameter %.9f in (%.9f mm), %zu attempts, %s\n",
                mcu ? "MCU" : "Host", input.c_str(), finalDiameter, finalDiameter * 25.4,
                attempts.size(), attempts.back().success ? "maximum reached" : "first error");
    if (!attempts.back().success)
      std::fprintf(stderr, "%s: stopped at %.9f in, code=%u line=%u: %s\n",
                   input.c_str(), attempts.back().diameterIn, attempts.back().diagnostic.code,
                   (unsigned)attempts.back().diagnostic.line, attempts.back().diagnostic.message.c_str());
  }
  if (!sweep_write_report(output / (mcu ? "mcu.sweep.html" : "host.sweep.html"), mcu, files))
    result = 1;
  return result;
}

static int sweep_self_test()
{
  const char *expectedMessages[] = {
      "Arc radius inconsistent", "Invalid move", "Move too short to compensate",
      "Arc radius less than tool radius", "Crossing error: move in cutting area",
      "Crossing error: move out of cutting area", "Unresolved gap between moves",
      "Input buffer overflow", "Output buffer overflow", "Too many consecutive Z moves",
      "Too many consecutive pauses", "Operation aborted"};
  for (size_t index = 0; index < sizeof(expectedMessages) / sizeof(expectedMessages[0]); ++index)
  {
    const cc_status_code_t code = static_cast<cc_status_code_t>(101 + index);
    SweepDiagnostic diagnostic;
    g_sweepDiagnostic = &diagnostic;
    host_xy_error_cb(code, CC_MSG_ERROR, 42);
    g_sweepDiagnostic = nullptr;
    if (!diagnostic.sawError || diagnostic.code != (unsigned)code ||
        diagnostic.line != 42 || diagnostic.message != expectedMessages[index])
    {
      std::fprintf(stderr, "MCU sweep error description failed for code %u\n", (unsigned)code);
      return 1;
    }
    const SweepFileResult file = {"error.nc", false, {{0.0001, false, diagnostic}}};
    std::ostringstream report;
    sweep_write_file_details(report, file);
    if (report.str().find(expectedMessages[index]) == std::string::npos ||
        report.str().find("MCU compensation error") != std::string::npos)
    {
      std::fprintf(stderr, "MCU sweep HTML error description failed for code %u\n", (unsigned)code);
      return 1;
    }
  }
  const std::vector<std::string> inchSquare = {
      "G20 G90 G0 X-2 Y0", "G1 G41 X0 Y0 F10", "X2 Y0", "X2 Y2",
      "X0 Y2", "X0 Y0", "G40 X-2 Y0"};
  const std::vector<std::string> mmSquare = {
      "G21 G90 G0 X-50.8 Y0", "G1 G41 X0 Y0 F254", "X50.8 Y0", "X50.8 Y50.8",
      "X0 Y50.8", "X0 Y0", "G40 X-50.8 Y0"};
  const std::vector<std::string> smallArc = {
      "G20 G90 G0 X-2 Y0", "G1 G41 X0 Y0 F10", "G3 X0.1 Y0.1 I0 J0.1",
      "G1 X0.1 Y1", "G40 X-2 Y1"};
  const std::vector<std::string> inchArc = {
      "G20 G90 G0 X-2 Y0", "G1 G41 X1 Y0 F10", "G3 X0 Y1 I-1 J0",
      "G1 X-1 Y1", "G40 X-2 Y1"};
  const std::vector<std::string> mmArc = {
      "G21 G90 G0 X-50.8 Y0", "G1 G41 X25.4 Y0 F254", "G3 X0 Y25.4 I-25.4 J0",
      "G1 X-25.4 Y25.4", "G40 X-50.8 Y25.4"};
  for (int engine = 0; engine < 2; ++engine)
  {
    const bool mcu = engine != 0;
    for (int units = 0; units < 2; ++units)
    {
      std::vector<Move2D> outputMoves;
      if (!sweep_attempt(units ? inchArc : mmArc, 0.5, mcu, &outputMoves).success)
      {
        std::fprintf(stderr, "Known-radius arc failed for engine %d units %d\n", engine, units);
        return 1;
      }
      bool foundExpectedRadius = false;
      for (const Move2D &move : outputMoves)
        if (move.type == MOT_ARC && std::fabs(move.radius - 19.05f) < 0.001f)
          foundExpectedRadius = true;
      if (!foundExpectedRadius)
      {
        std::fprintf(stderr, "Diameter/2 unit conversion did not produce a 19.05 mm arc for engine %d units %d\n",
                     engine, units);
        return 1;
      }
    }
    const std::vector<SweepAttempt> inches = sweep_program(inchSquare, mcu);
    const std::vector<SweepAttempt> millimeters = sweep_program(mmSquare, mcu);
    if (inches.size() != millimeters.size() || inches.size() != 826 ||
        sweep_increment_count(inches) != 825 ||
        inches.front().diameterIn != SWEEP_START_IN || inches.back().diameterIn != SWEEP_MAX_IN)
    {
      std::fprintf(stderr, "Sweep endpoints, count or unit equivalence failed for engine %d\n", engine);
      return 1;
    }
    for (size_t index = 0; index < inches.size(); ++index)
    {
      if (!inches[index].success || !millimeters[index].success ||
          inches[index].diameterIn != millimeters[index].diameterIn ||
          (index > 0 && inches[index].diameterIn != sweep_next_diameter(inches[index - 1].diameterIn)))
      {
        std::fprintf(stderr, "Sweep progression/unit conversion failed at attempt %zu engine %d\n", index + 1, engine);
        return 1;
      }
    }
    const std::vector<SweepAttempt> limited = sweep_program(smallArc, mcu);
    if (limited.size() < 2 || limited.back().success ||
        !limited[limited.size() - 2].success || limited.back().diagnostic.code == 0 ||
        limited.back().diagnostic.line == 0 ||
        limited[limited.size() - 2].diameterIn > 0.2 || limited.back().diameterIn <= 0.2)
    {
      std::fprintf(stderr, "Sweep arc limit failed for engine %d: attempts=%zu last=%.9f in "
                   "previous=%.9f in code=%u line=%u message=%s\n",
                   engine, limited.size(), limited.back().diameterIn,
                   limited.size() > 1 ? limited[limited.size() - 2].diameterIn : 0.0,
                   limited.back().diagnostic.code, (unsigned)limited.back().diagnostic.line,
                   limited.back().diagnostic.message.c_str());
      return 1;
    }
    const std::vector<SweepAttempt> firstFailure = {sweep_attempt(smallArc, SWEEP_MAX_IN, mcu)};
    if (firstFailure.front().success || sweep_increment_count(firstFailure) != 0 ||
        !sweep_attempt(inchSquare, SWEEP_START_IN, mcu).success)
    {
      std::fprintf(stderr, "Sweep failed first-attempt rejection or reset for engine %d\n", engine);
      return 1;
    }
  }
  if (sweep_html_escape("<&\">") != "&lt;&amp;&quot;&gt;")
  {
    std::fprintf(stderr, "Sweep HTML escaping failed\n");
    return 1;
  }
  puts("Host and MCU diameter sweep self-tests passed");
  return 0;
}
