# Cutter_Comp_XY

2D cutter compensation engine for XY toolpaths with both Arduino-target and desktop-host execution paths.

## What’s New
- Added host-side file-driven workflow (`data/*.nc`) for fast iteration without flashing hardware.
- Added incremental coordinate support (`G91`) in addition to absolute (`G90`).
- Added arc center resolution from either `I/J` or radius `R` for `G2/G3`.
- Added post-pass crossing trim with configurable multi-pass lookahead.
- Added colinear segment merge cleanup after compensation.
- Added SVG and compensated G-code output generation from the host harness.

## Current Feature Set
- Cutter compensation modes: `G41` (left), `G42` (right), `G40` (cancel).
- Motion support: `G0`, `G1`, `G2`, `G3` in XY (`G17`).
- Coordinate modes: `G90` absolute and `G91` incremental.
- Arc input formats: `I/J` incremental center offsets and `R`-based arc definition.
- Comment stripping support for both `;` and `( ... )` comments.
- Optional corner rolling / fillet-style transition behavior.
- Optional self-intersection trimming and geometry cleanup pass.

## Project Layout
- `src/` - Core parser + compensation engine (`SimpleGCodeScan.h`, `CutterComp2D.h`).
- `host/` - Desktop harness (`main_host.cpp`) and output writers (`writer.h`).
- `data/` - Sample NC programs used by the host harness.
- `build/` - CMake/Visual Studio build output (generated).
- `platformio.ini` - Arduino board environments.

## Host Workflow (Recommended for development)
1. Build `cuttercomp_host` (CMake/VS task).
2. Run the executable.
3. The program loads one NC file (configured in `host/main_host.cpp`) and runs compensation.
4. Generated outputs:
	 - `out.ngc` - compensated toolpath G-code
	 - `out.svg` - visual overlay (original vs compensated)

Input examples are provided under `data/` (`G41_1.nc`, `G42_1.nc`, `TortureTestG90.nc`, `TortureTestG91.nc`, etc.).

## Arduino / PlatformIO
- Configured environments are defined in `platformio.ini`:
	- `due`
	- `adafruit_grandcentral_m4`
- Embedded entry point is `src/main.cpp`.

## Notes and Scope
- This project focuses on 2D XY compensation behavior and geometry handling.
- The host harness is for algorithm validation and visualization; it is not a machine controller.

## Cutter Comp Types in Fusion360
- In computer 
Tool compensation is calculated automatically by the program

 - In control - Tool compensation is not calculated, but rather G41/G42 codes are output to allow the operator to set the compensation amount and wear on the machine tool control.

 - Wear - Works as if In computer was selected, but also outputs the G41/G42 codes. This lets the machine tool operator adjust tool wear at the machine tool control by entering the difference in tool size as a negative number.

 - Inverse wear - Identical to the Wear option, except that the wear adjustment is entered as a positive number.

## License
MIT
