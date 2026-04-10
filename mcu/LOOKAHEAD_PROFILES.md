# MCU Look-Ahead Profiles

These are RAM-budgeted compile-time presets for the MCU cutter comp look-ahead path.

Assumption used for rough sizing:
- move2d is about 64 bytes on target

Approximate look-ahead memory:
- Persistent context RAM: 64 x CC_LOOKAHEAD_CAP bytes
- Temporary trim stack RAM: 16 x CC_LOOKAHEAD_CAP bytes
- Plus a small amount for counters/locals

## Small RAM profile (tight MCU, current default)
Good for very limited SRAM while still enabling crossing trim. This matches the current defaults in `cutter_comp.h`.

```c
#define CC_ENABLE_LOOKAHEAD 1
#define CC_LOOKAHEAD_CAP 8
#define CC_LOOKAHEAD_STEPS 4
#define CC_LA_TARGET_BATCH_EMIT 1
#define CC_LA_TRIM_OVERLAP (CC_LOOKAHEAD_STEPS + 2)
#define CC_LA_EMIT_HOLDBACK CC_LA_TRIM_OVERLAP
#define CC_LA_MIN_PENDING (CC_LA_EMIT_HOLDBACK + CC_LA_TARGET_BATCH_EMIT)
```

Approx memory:
- Context: about 512 B
- Trim stack: about 128 B

## Medium profile (balanced)
Good for typical MCU SRAM budgets.

```c
#define CC_ENABLE_LOOKAHEAD 1
#define CC_LOOKAHEAD_CAP 16
#define CC_LOOKAHEAD_STEPS 8
#define CC_LA_TARGET_BATCH_EMIT 6
#define CC_LA_TRIM_OVERLAP (CC_LOOKAHEAD_STEPS + 2)
#define CC_LA_EMIT_HOLDBACK CC_LA_TRIM_OVERLAP
#define CC_LA_MIN_PENDING (CC_LA_EMIT_HOLDBACK + CC_LA_TARGET_BATCH_EMIT)
```

Approx memory:
- Context: about 1024 B
- Trim stack: about 256 B

## Larger profile (quality first)
Good when SRAM is available and you want stronger crossing handling.

```c
#define CC_ENABLE_LOOKAHEAD 1
#define CC_LOOKAHEAD_CAP 24
#define CC_LOOKAHEAD_STEPS 12
#define CC_LA_TARGET_BATCH_EMIT 10
#define CC_LA_TRIM_OVERLAP (CC_LOOKAHEAD_STEPS + 2)
#define CC_LA_EMIT_HOLDBACK CC_LA_TRIM_OVERLAP
#define CC_LA_MIN_PENDING (CC_LA_EMIT_HOLDBACK + CC_LA_TARGET_BATCH_EMIT)
```

Approx memory:
- Context: about 1536 B
- Trim stack: about 384 B
