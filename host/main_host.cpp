#include <cstdio>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <cmath>

  #define DBG_PRINTLN(x) std::puts(x)
  #define DBG_PRINT(x, ...) std::printf(x, ##__VA_ARGS__)

// If any of your headers include <Arduino.h>, include the compat first and
// make sure your headers include ArduinoCompat.h when not ARDUINO.
#include "ArduinoCompat.h"

#include "TinyGCodeScan.h"
#include "CutterComp2D.h"
#include "TestData.h"

/*
  This is a desktop test harness for the CutterComp2D class, which performs 2D cutter compensation on linear and arc moves.

  It includes a post-pass to trim crossing elements, which is a common source of tiny unwanted moves after compensation. This is optional and can be toggled with ENABLE_TRIM_CROSSINGS.

  The test uses hardcoded G-code in TestData.h, which you can modify to test different scenarios. The output is printed as G-code lines, and also saved to "output.csv" for analysis and "output.svg" for visualization.

  Note: This code is meant for testing the compensation logic on the host. It does not run on an Arduino or control any hardware.
*/

// -------------------- Config --------------------
static constexpr float TOOL_RADIUS = 0.0625f;
static constexpr bool ENABLE_COMP   = true;
static constexpr bool ENABLE_FILLET = true;
static constexpr bool ENABLE_TRIM_CROSSINGS = true;

static constexpr int MAX_LOOKAHEAD_FOR_INTERSECTIONS = 25;
static constexpr int MAX_TRIM_PASSES = 6;
// ------------------------------------------------

static ModalState modal;
static CutterComp2D cc;

// -------------------- Profile buffer --------------------
static std::vector<Move2D> profile;

static void profile_reset() { profile.clear(); }

static void profile_push(const Move2D& m)
{
  Move2D t = m;
  t.valid = true;
  profile.push_back(t);
}

// -------------------- Emit helpers (host files) --------------------
static void emit_move_as_gcode(FILE* f, const Move2D& m)
{
  if (m.type == MOT_LINE) {
    std::fprintf(f, "N%d %s X%.4f Y%.4f\n",
                 (int)m.seqNum, m.rapid ? "G0" : "G1", m.p1.x, m.p1.y);
    return;
  }

  if (m.type == MOT_ARC) {
    float I = m.center.x - m.p0.x;
    float J = m.center.y - m.p0.y;
    std::fprintf(f, "N%d %s X%.4f Y%.4f I%.4f J%.4f\n",
                 (int)m.seqNum, (m.arcDir == ARC_CW) ? "G2" : "G3",
                 m.p1.x, m.p1.y, I, J);
    return;
  }
}


// -------------------- SVG writer --------------------
struct Bounds {
  float minx=+1e30f, miny=+1e30f, maxx=-1e30f, maxy=-1e30f;
  void add(Vec2 p) {
    minx = std::min(minx, p.x); miny = std::min(miny, p.y);
    maxx = std::max(maxx, p.x); maxy = std::max(maxy, p.y);
  }
};

static void svg_polyline(std::ostringstream& ss, const std::vector<Vec2>& pts, const char* stroke)
{
  if (pts.size() < 2) return;
  ss << "<polyline fill=\"none\" stroke=\"" << stroke << "\" stroke-width=\"0.002\" points=\"";
  for (auto& p : pts) ss << p.x << "," << p.y << " ";
  ss << "\" />\n";
}

static std::vector<Vec2> approx_move_points(const Move2D& m, int arcSegments=24)
{
  std::vector<Vec2> pts;

  if (m.type == MOT_LINE) {
    pts.push_back(m.p0);
    pts.push_back(m.p1);
    return pts;
  }

  if (m.type == MOT_ARC) {
    float a0 = std::atan2(m.p0.y - m.center.y, m.p0.x - m.center.x);
    float a1 = std::atan2(m.p1.y - m.center.y, m.p1.x - m.center.x);

    auto norm = [](float a){
      while (a < 0) a += 2.0f*(float)M_PI;
      while (a >= 2.0f*(float)M_PI) a -= 2.0f*(float)M_PI;
      return a;
    };
    a0 = norm(a0); a1 = norm(a1);

    float sweep;
    if (m.arcDir == ARC_CCW) {
      sweep = a1 - a0; if (sweep < 0) sweep += 2.0f*(float)M_PI;
    } else {
      sweep = a0 - a1; if (sweep < 0) sweep += 2.0f*(float)M_PI;
      sweep = -sweep; // negative for CW stepping
    }

    int n = std::max(6, arcSegments);
    pts.reserve(n+1);

    for (int i=0; i<=n; ++i) {
      float t = (float)i / (float)n;
      float a = a0 + sweep * t;
      Vec2 p = v2(m.center.x + m.radius*std::cos(a),
                  m.center.y + m.radius*std::sin(a));
      pts.push_back(p);
    }
    return pts;
  }

  return pts;
}

static void write_svg(const char* path,
                      const std::vector<Move2D>& moves,
                      const std::vector<Move2D>* original = nullptr,
                      bool mirror_x = false,
                      bool mirror_y = false)
{

  // Pass 1: Compute bounds using unmirrored points
  Bounds b;
  auto accumulate_bounds = [&](const std::vector<Move2D>& mv, bool onlyValid){
    for (auto& m : mv) {
      if (onlyValid && (!m.valid || m.type==MOT_EMPTY)) continue;
      auto pts = approx_move_points(m);
      for (auto& p : pts) {
        b.add(p);
      }
    }
  };
  if (original) accumulate_bounds(*original, false);
  accumulate_bounds(moves, true);

  float pad = 0.05f * std::max( (b.maxx-b.minx), (b.maxy-b.miny) );
  float minx = b.minx - pad, miny = b.miny - pad;
  float w = (b.maxx - b.minx) + 2*pad;
  float h = (b.maxy - b.miny) + 2*pad;

  std::ostringstream ss;
  ss << "<svg xmlns=\"http://www.w3.org/2000/svg\" viewBox=\""
     << minx << " " << miny << " " << w << " " << h << "\">\n";
  ss << "<rect x=\"" << minx << "\" y=\"" << miny << "\" width=\"" << w
     << "\" height=\"" << h << "\" fill=\"white\" />\n";

  // Original (blue) if provided
  if (original) {
    for (auto& m : *original) {
      if (m.type==MOT_EMPTY || m.rapid) continue;
      auto pts = approx_move_points(m);
      for (auto& p : pts) {
        if (mirror_x) p.x = b.maxx + b.minx - p.x;
        if (mirror_y) p.y = b.maxy + b.miny - p.y;
      }
      svg_polyline(ss, pts, "#1f77b4");
    }
  }

  // Output (red)
  for (auto& m : moves) {
    if (!m.valid || m.type==MOT_EMPTY || m.rapid) continue;
    auto pts = approx_move_points(m);
    for (auto& p : pts) {
      if (mirror_x) p.x = b.maxx + b.minx - p.x;
      if (mirror_y) p.y = b.maxy + b.miny - p.y;
    }
    svg_polyline(ss, pts, "#d62728");
  }

  ss << "</svg>\n";

  std::ofstream out(path, std::ios::binary);
  out << ss.str();
}

// -------------------- Pipeline --------------------
static void process_one_gcode_line(const char* raw)
{
  char clean[160];
  strip_comments(raw, clean, sizeof(clean));

  const char* p = clean;
  while (*p == ' ' || *p == '\t') ++p;
  if (*p == 0) return;

  ScanLine s;
  scan_line(clean, s);

  Move2D mv = interpret_to_move(s, modal);
 
  if(mv.type==MOT_RAPID) {
    // should never have rapid moves.
    return;
  }

  if (ENABLE_COMP && (s.sawG41 || s.sawG42)) {
    cc.setComp(modal.comp); // before processing this move
  }

  if (mv.type == MOT_EMPTY) {
    if (ENABLE_COMP && s.sawG40) {
      cc.setComp(COMP_OFF);
      cc.flush();
      Move2D out;
      while (cc.popOut(out)) profile_push(out);
    }
    return;
  }

  if (!ENABLE_COMP) {
    profile_push(mv);
  } else {
    if (!cc.pushIn(mv)) {
      std::puts("(comp input buffer full)");
      return;
    }
    cc.process(true); // force rolling for demo

    Move2D out;
    while (cc.popOut(out)) profile_push(out);
  }

  if (ENABLE_COMP && s.sawG40) {
    //cc.setComp(COMP_OFF);
    cc.flush();
    Move2D out;
    while (cc.popOut(out)) profile_push(out);
  }
}

static void flush_pipeline()
{
  if (!ENABLE_COMP) return;
  cc.flush();
  Move2D out;
  while (cc.popOut(out)) profile_push(out);
}

// Build a simple "original moves" list (no comp) for overlay.
static std::vector<Move2D> build_original_moves()
{
  std::vector<Move2D> orig;
  ModalState m{};
  m.planeXY = true; m.absXY = true; m.motionG = 0; m.comp = COMP_OFF; m.feed=0; m.pos=v2(0,0);

  const int lines = (int)(sizeof(demo_program) / sizeof(demo_program[0]));
  for (int i=0; i<lines; ++i) {
    char clean[160];
    strip_comments(demo_program[i], clean, sizeof(clean));
    ScanLine s; scan_line(clean, s);
    Move2D mv = interpret_to_move(s, m);
    if (mv.type != MOT_EMPTY) {
      mv.valid = true;
      orig.push_back(mv);
    }
  }
  return orig;
}

int main()
{
  // Init modal
  modal = ModalState{};
  modal.planeXY = true;
  modal.absXY   = true;
  modal.motionG = 0;
  modal.comp    = COMP_OFF;
  modal.feed    = 0;
  modal.pos     = v2(0, 0);

  // Init cutter comp
  cc.setToolRadius(TOOL_RADIUS);
  cc.setCornerRolling(ENABLE_FILLET);
  cc.setComp(COMP_OFF);

  profile_reset();

  const int lines = (int)(sizeof(demo_program) / sizeof(demo_program[0]));
  for (int i = 0; i < lines; ++i) {
    process_one_gcode_line(demo_program[i]);
  }
  flush_pipeline();

  // Post-pass trim crossings (optional)
  if (ENABLE_TRIM_CROSSINGS) {
    for (int pass = 0; pass < MAX_TRIM_PASSES; ++pass) {
      bool changed = cc.trimCrossingElements(profile.data(), (int)profile.size(),
                                          MAX_LOOKAHEAD_FOR_INTERSECTIONS);
      if (!changed) break;
    }
  }

  // (Optional) later fixup for CM_IN/CM_OUT
  cc.fixup_comp_in_out(profile.data(), (int)profile.size());

  // Write outputs
  auto orig = build_original_moves();
  write_svg("out.svg", profile, &orig,false,true);// mirror for better visualization

  // G-code file
  if (FILE* f = std::fopen("out.ngc", "wb")) {
    for (auto& m : profile) {
      if (!m.valid || m.type==MOT_EMPTY) continue;
      emit_move_as_gcode(f, m);
    }
    std::fclose(f);
  }

  //write_csv("out.csv");

  std::puts("Wrote: out.svg, out.ngc, out.csv");
  return 0;
}
