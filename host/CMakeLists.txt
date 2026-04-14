cmake_minimum_required(VERSION 3.16)
project(cc_runner LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(GRBL_HOST_AVAILABLE OFF)
if (EXISTS ${CMAKE_CURRENT_LIST_DIR}/../grbl/CMakeLists.txt
    AND EXISTS ${CMAKE_CURRENT_LIST_DIR}/grbl_host_main.c
    AND EXISTS ${CMAKE_CURRENT_LIST_DIR}/grbl_host_driver.c
    AND EXISTS ${CMAKE_CURRENT_LIST_DIR}/grbl_host_stubs.c)
  set(GRBL_HOST_AVAILABLE ON)

  add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../grbl ${CMAKE_CURRENT_BINARY_DIR}/grbl)

  set(GRBL_HOST_CORE_SOURCES
    ${CMAKE_CURRENT_LIST_DIR}/../grbl/grbllib.c
    ${CMAKE_CURRENT_LIST_DIR}/../grbl/coolant_control.c
    ${CMAKE_CURRENT_LIST_DIR}/../grbl/crossbar.c
    ${CMAKE_CURRENT_LIST_DIR}/../grbl/crc.c
    ${CMAKE_CURRENT_LIST_DIR}/../grbl/nvs_buffer.c
    ${CMAKE_CURRENT_LIST_DIR}/../grbl/gcode.c
    ${CMAKE_CURRENT_LIST_DIR}/../grbl/machine_limits.c
    ${CMAKE_CURRENT_LIST_DIR}/../grbl/messages.c
    ${CMAKE_CURRENT_LIST_DIR}/../grbl/motion_control.c
    ${CMAKE_CURRENT_LIST_DIR}/../grbl/nuts_bolts.c
    ${CMAKE_CURRENT_LIST_DIR}/../grbl/override.c
    ${CMAKE_CURRENT_LIST_DIR}/../grbl/planner.c
    ${CMAKE_CURRENT_LIST_DIR}/../grbl/probe.c
    ${CMAKE_CURRENT_LIST_DIR}/../grbl/protocol.c
    ${CMAKE_CURRENT_LIST_DIR}/../grbl/report.c
    ${CMAKE_CURRENT_LIST_DIR}/../grbl/settings.c
    ${CMAKE_CURRENT_LIST_DIR}/../grbl/sleep.c
    ${CMAKE_CURRENT_LIST_DIR}/../grbl/spindle_control.c
    ${CMAKE_CURRENT_LIST_DIR}/../grbl/state_machine.c
    ${CMAKE_CURRENT_LIST_DIR}/../grbl/stream.c
    ${CMAKE_CURRENT_LIST_DIR}/../grbl/stream_file.c
    ${CMAKE_CURRENT_LIST_DIR}/../grbl/stream_passthru.c
    ${CMAKE_CURRENT_LIST_DIR}/../grbl/stepper.c
    ${CMAKE_CURRENT_LIST_DIR}/../grbl/stepper2.c
    ${CMAKE_CURRENT_LIST_DIR}/../grbl/strutils.c
    ${CMAKE_CURRENT_LIST_DIR}/../grbl/system.c
    ${CMAKE_CURRENT_LIST_DIR}/../grbl/tool_change.c
    ${CMAKE_CURRENT_LIST_DIR}/../grbl/alarms.c
    ${CMAKE_CURRENT_LIST_DIR}/../grbl/errors.c
    ${CMAKE_CURRENT_LIST_DIR}/../grbl/ngc_params.c
    ${CMAKE_CURRENT_LIST_DIR}/../grbl/ngc_expr.c
    ${CMAKE_CURRENT_LIST_DIR}/../grbl/ngc_flowctrl.c
    ${CMAKE_CURRENT_LIST_DIR}/../grbl/regex.c
    ${CMAKE_CURRENT_LIST_DIR}/../grbl/utf8.c
    ${CMAKE_CURRENT_LIST_DIR}/../grbl/vfs.c
  )
endif()

add_executable(cc_runner
  main_host.cpp
  ../mcu/cutter_comp.c
)

if (GRBL_HOST_AVAILABLE)
  add_executable(grbl_host_runner
    grbl_host_main.c
    grbl_host_driver.c
    grbl_host_stubs.c
    ../mcu/cutter_comp.c
    ${GRBL_HOST_CORE_SOURCES}
  )
endif()

# Adjust this include path list to match your repo.
# Typical PlatformIO layout:
#   include/  (headers)
#   src/      (headers + code)
target_include_directories(cc_runner PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}        # host/
  ${CMAKE_CURRENT_LIST_DIR}/../src # <-- change if your headers aren't in src
  ${CMAKE_CURRENT_LIST_DIR}/../include
)

if (GRBL_HOST_AVAILABLE)
  target_include_directories(grbl_host_runner PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/../grbl
    ${CMAKE_CURRENT_LIST_DIR}/../grbl/kinematics
    ${CMAKE_CURRENT_LIST_DIR}/../src
    ${CMAKE_CURRENT_LIST_DIR}/../include
  )
endif()

# Optional: warnings help a lot on desktop
if (MSVC)
  target_compile_options(cc_runner PRIVATE /W4)
else()
  target_compile_options(cc_runner PRIVATE -Wall -Wextra -Wpedantic)
endif()

# If your code uses math constants on MSVC:
if (MSVC)
  target_compile_definitions(cc_runner PRIVATE _USE_MATH_DEFINES)
endif()

target_compile_definitions(cc_runner PRIVATE
  _USE_MATH_DEFINES
  CUTTER_COMP_ENABLE=1
)

if (GRBL_HOST_AVAILABLE)
  target_compile_definitions(grbl_host_runner PRIVATE
    _USE_MATH_DEFINES
    _CRT_SECURE_NO_WARNINGS
    CUTTER_COMP_ENABLE=1
  )
endif()

if (GRBL_HOST_AVAILABLE)
  if (MSVC)
    target_compile_options(grbl_host_runner PRIVATE /W4)
  else()
    target_compile_options(grbl_host_runner PRIVATE -Wall -Wextra -Wpedantic)
  endif()
endif()
