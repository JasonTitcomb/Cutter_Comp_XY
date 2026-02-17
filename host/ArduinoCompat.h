#pragma once

// Minimal Arduino compatibility layer for desktop builds.

#include <cstdint>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <cstring>

#ifndef PROGMEM
#define PROGMEM
#endif

// If your code uses Arduino-style functions, stub them here.
static inline void delay(unsigned) {}

// Optional: replace Serial prints with stdout
struct SerialLike {
  void begin(uint32_t) {}
  void println() { std::puts(""); }
  void println(const char* s) { std::puts(s); }

  void print(const char* s) { std::fputs(s, stdout); }
  void print(char c) { std::fputc(c, stdout); }

  void print(int v) { std::printf("%d", v); }
  void print(unsigned v) { std::printf("%u", v); }
  void print(long v) { std::printf("%ld", v); }
  void print(unsigned long v) { std::printf("%lu", v); }

  void print(float v, int digits = 2) { std::printf("%.*f", digits, v); }
  void print(double v, int digits = 2) { std::printf("%.*f", digits, v); }

  template <typename T>
  void println(T v) { print(v); std::puts(""); }
};

static SerialLike Serial;
