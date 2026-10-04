#pragma once
#include <vector>
#include "../src/cc_math.h"

struct ComparePair
{
  size_t fullIndex;
  size_t simpleIndex;
};

static bool compare_moves_match(const Move2D &full, const Move2D &simple, float tolerance)
{
  if (full.type != simple.type || full.valid != simple.valid ||
      dist(full.p_0, simple.p_0) > tolerance || dist(full.p_1, simple.p_1) > tolerance)
    return false;
  return full.type != MOT_ARC ||
         (full.arcDir == simple.arcDir && dist(full.center, simple.center) <= tolerance &&
          fabsf(full.radius - simple.radius) <= tolerance);
}

static std::vector<ComparePair> align_compare_moves(const std::vector<Move2D> &full,
                                                   const std::vector<Move2D> &simple,
                                                   float tolerance)
{
  std::vector<ComparePair> pairs;
  pairs.reserve(full.size() + simple.size());
  size_t fullIndex = 0;
  size_t simpleIndex = 0;
  const size_t lookahead = 16;
  while (fullIndex < full.size() && simpleIndex < simple.size())
  {
    if (!compare_moves_match(full[fullIndex], simple[simpleIndex], tolerance))
    {
      size_t skipFull = 0;
      size_t skipSimple = 0;
      bool found = false;
      for (size_t skipped = 1; skipped <= lookahead && !found; ++skipped)
      {
        if (fullIndex + skipped < full.size() && simpleIndex + skipped < simple.size() &&
            compare_moves_match(full[fullIndex + skipped], simple[simpleIndex + skipped], tolerance))
          break;
        for (size_t fullOffset = 0; fullOffset <= skipped; ++fullOffset)
        {
          const size_t simpleOffset = skipped - fullOffset;
          if (fullIndex + fullOffset < full.size() && simpleIndex + simpleOffset < simple.size() &&
              compare_moves_match(full[fullIndex + fullOffset], simple[simpleIndex + simpleOffset], tolerance))
          {
            skipFull = fullOffset;
            skipSimple = simpleOffset;
            found = true;
            break;
          }
        }
      }
      for (size_t skipped = 0; skipped < skipFull; ++skipped)
        pairs.push_back({fullIndex++, simple.size()});
      for (size_t skipped = 0; skipped < skipSimple; ++skipped)
        pairs.push_back({full.size(), simpleIndex++});
    }
    pairs.push_back({fullIndex++, simpleIndex++});
  }
  while (fullIndex < full.size())
    pairs.push_back({fullIndex++, simple.size()});
  while (simpleIndex < simple.size())
    pairs.push_back({full.size(), simpleIndex++});
  return pairs;
}
