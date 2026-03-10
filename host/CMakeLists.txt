cmake_minimum_required(VERSION 3.16)
project(cuttercomp_host LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(cuttercomp_host
  main_host.cpp
  ../src/cc_main.cpp
)

# Adjust this include path list to match your repo.
# Typical PlatformIO layout:
#   include/  (headers)
#   src/      (headers + code)
target_include_directories(cuttercomp_host PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}        # host/
  ${CMAKE_CURRENT_LIST_DIR}/../src # <-- change if your headers aren't in src
  ${CMAKE_CURRENT_LIST_DIR}/../include
)

# Optional: warnings help a lot on desktop
if (MSVC)
  target_compile_options(cuttercomp_host PRIVATE /W4)
else()
  target_compile_options(cuttercomp_host PRIVATE -Wall -Wextra -Wpedantic)
endif()

# If your code uses math constants on MSVC:
if (MSVC)
  target_compile_definitions(cuttercomp_host PRIVATE _USE_MATH_DEFINES)
endif()
