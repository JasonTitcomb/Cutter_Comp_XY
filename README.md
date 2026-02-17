# Cutter_Comp_XY

This project implements 2D cutter compensation for CNC toolpaths, supporting both linear and arc moves. It includes a desktop test harness for simulation and visualization.

## Features
- 2D cutter compensation (G41/G42)
- Arc and line move support
- Post-pass trimming of crossing elements
- SVG and CSV output for visualization and analysis
- Host simulation (does not run on Arduino)

## Structure
- `src/` - Core compensation logic and data structures
- `host/` - Desktop test harness and visualization tools
- `include/` - Shared headers
- `lib/` - External libraries (if any)
- `test/` - Test data and scripts
- `build/` - Build artifacts

## Usage
1. Build and run the host application to simulate toolpath compensation.
2. Outputs are generated as SVG, CSV, and G-code files for review.

## Requirements
- C++ compiler (Visual Studio recommended)
- PlatformIO for Arduino compatibility

## License
MIT License

---
For questions or contributions, please open an issue or pull request.
