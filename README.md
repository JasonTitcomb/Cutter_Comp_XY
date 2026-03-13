# Cutter_Comp_XY
⚠️ Safety Notice

This is hobby-grade experimental code developed for personal CNC equipment.
It has not been validated for safety-critical use.
Running motion-control software can damage machines or cause injury if used
incorrectly. Review the code and test carefully before using it on real hardware.

2D cutter compensation engine for XY toolpaths with both Arduino-target and desktop-host execution paths.


## Current Feature Set
- Cutter compensation modes: `G41` (left), `G42` (right), `G40` (cancel).
- Motion support: `G0`, `G1`, `G2`, `G3` in XY (`G17`).
- Coordinate modes: `G90` absolute and `G91` incremental.
- Arc input formats: `I/J` incremental center offsets and `R`-based arc definition.
- Comment stripping support for both `;` and `( ... )` comments.
- Corner rolling / bevel-style transition behavior.
- Optional global self-intersection trimming and geometry cleanup pass.
- Configurable look ahead for gouge detection and buffer sizing.
- +- offset values supported for wear compensation.
- Incremental/Absolute support.
- Z allowed but not used for calculations.
- Global self intersections are ignored when Z does not match between pairwise comparisons to allow for thread milling.

## Project Layout
- `src/` - Core parser + compensation engine (`cc_simple_scan.h`, `cc_processor.h`, `cc_main.h`, `cc_main.cpp`).
- `host/` - Desktop harness (`main_host.cpp`) and output writers (`writer.h`).
- `data/` - Sample NC programs used by the host harness.
- `output/` - Generated host outputs (`.ngc`, `.svg`).
- `build/` - CMake/Visual Studio build output (generated).
- `platformio.ini` - Arduino board environments.

## Host Workflow (Recommended for development)
1. Build `cuttercomp_host` (CMake/VS task).
2. Run the executable.
3. The program loads one NC file (configured in `host/main_host.cpp`) and runs compensation.
4. Generated outputs:
	 - `output/<input>.ngc` - compensated toolpath G-code
	 - `output/<input>.svg` - visual overlay (original vs compensated)

Input examples are provided under `data/` (`G41_1.nc`, `G42_1.nc`, `TortureTestG90.nc`, `TortureTestG91.nc`, etc.).

## Arduino / PlatformIO
- Configured environments are defined in `platformio.ini`:
	- `due`
	- `adafruit_grandcentral_m4`
- Embedded entry point is `src/main.cpp`.

## Integration API
- High-level runtime wrapper: `CcMainRunner` in `src/cc_main.h`.
- Callback signatures:
	- `CcOutputCB(const char *text, size_t len)`
	- `CcErrorCB(const char *message, CompError err, uint32_t seqNum)`
- `CcMainCallbacks` contains `output` and `error` callbacks only (no `userData`).

## Notes and Scope
- This project focuses on 2D XY compensation behavior and geometry handling.
- The host harness is for algorithm validation and visualization; it is not a machine controller.


## 1. Full Tool Radius / Diameter Compensation ("Control" or "In Control" comp)

Tool offset table → Enter the full tool radius (or full diameter, depending on control parameter setting).
CAM post → Outputs toolpath at part geometry (centerline of feature), then adds G41/G42.
Machine → Offsets the path by the full amount in the offset table.
Pros — Simple if you always measure and enter the exact tool size.
Cons — Lead-in/lead-out moves must be longer than the tool radius (to avoid alarms/interference). Changing tools requires updating the full value. Small adjustments (e.g., 0.0005" for size tweaking) can cause unexpected over/under-cutting if not careful.

## 2. Wear Compensation (Most popular in modern shops with CAM)

Tool offset table → Enter 0 (or very close to the nominal tool radius/diameter) in the geometry/radius column.
CAM post → Outputs toolpath offset by the nominal tool radius (tool centerline path), still with G41/G42.
Machine → The offset value (wear column or sometimes the same D register) adds/subtracts a small deviation.
Positive wear → Tool moves away from part (less material removed, bigger part).
Negative wear → Tool moves toward part (more material removed, smaller part).

## Corner Treatment
Corners can be a arc roll or a chamfer style.

![chamfer style](images/chamfers.png)

![roll style](images/roll.png)

Internal corners are consumed if needed.

![internal corners](images/internalcorners.png)

## License
MIT
