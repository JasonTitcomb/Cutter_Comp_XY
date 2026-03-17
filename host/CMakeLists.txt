cmake_minimum_required(VERSION 3.16)
project(cc_runner LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(cc_runner
  main_host.cpp
  ../mcu/cutter_comp.c
)

# Adjust this include path list to match your repo.
# Typical PlatformIO layout:
#   include/  (headers)
#   src/      (headers + code)
target_include_directories(cc_runner PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}        # host/
  ${CMAKE_CURRENT_LIST_DIR}/../src # <-- change if your headers aren't in src
  ${CMAKE_CURRENT_LIST_DIR}/../include
)

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
