# Embedded Cutter Compensation Flow

This document captures the recommended embedded runtime flow for `CcMainRunner` (which wraps `CutterComp2D`) based on the current host-side behavior.

## High-level flow

- Initialize parser modal state with `ModalState{}` and defaults (`planeXY`, `absXYZ`, `motionG`, `comp`, `feed`, `speed`, `pos`).
- Configure `CutterComp2D` with existing APIs only: `setToolRadius`, `setCornerTreatment`, `setErrorCallback`, `setComp(COMP_OFF)`.
- For each raw G-code line:
  - Strip comments: `strip_comments(raw, clean, sizeof(clean))`.
  - Scan tokens: `scan_line(clean, s)`.
  - Interpret move and update modal state: `Move2D mv = interpret_move(s, modalState)`.
  - If line enables comp (`G41`/`G42`), call `cc.setComp(modalState.comp)` before processing that move.
  - If `mv.type == MOT_EMPTY`, handle optional non-motion `G40` shutdown path and return.
  - Push exactly one move with `pushIn`; if full, treat as input-buffer fault.
  - Call `process` once; if false, treat as output-space/processing failure.
  - Drain all available output via `popOut` and push to profile/planner buffer.
  - If this move carries `G40`, call `flush`, then `setComp(COMP_OFF)`, then drain `popOut` again.
- At program end, call final `flush` and drain `popOut`.

## Preferred embedded entrypoint

Use `CcMainRunner` for embedded/firmware integration unless you are intentionally testing low-level `CutterComp2D` behavior.

```cpp
CcMainRunner runner;
CcMainOptions options;
CcMainCallbacks callbacks;

callbacks.output = serial_output_cb;
callbacks.error = serial_error_cb;

bool ok = runner.begin(options, callbacks);
while (ok && read_next_gcode_line(raw))
    ok = runner.processLine(raw);

if (ok)
    ok = runner.finish();
```

Callback signatures in current code:
- `void serial_output_cb(const char *text, size_t len)`
- `void serial_error_cb(const char *message, CompError err, uint32_t seqNum)`

## Firmware-style pseudocode

```cpp
CutterComp2D cc;
ModalState modalState = ModalState{};
modalState.planeXY = true;
modalState.absXYZ = true;
modalState.motionG = 0;
modalState.comp = COMP_OFF;
modalState.feed = 0.0f;
modalState.speed = 0.0f;
modalState.pos = v2(0, 0);

cc.setToolRadius(TOOL_RADIUS);
cc.setCornerTreatment(CORNER_TREATMENT);
cc.setErrorCallback(compErrorHandler);
cc.setComp(COMP_OFF);

while (read_next_gcode_line(raw))
{
    char clean[160];
    strip_comments(raw, clean, sizeof(clean));

    ScanLine s;
    scan_line(clean, s);
    Move2D mv = interpret_move(s, modalState);

    if (s.sawG41 || s.sawG42)
        cc.setComp(modalState.comp); // set before processing current move

    if (mv.type == MOT_EMPTY)
    {
        if (s.sawG40)
        {
            cc.setComp(COMP_OFF);
            cc.flush();

            Move2D out;
            while (cc.popOut(out))
                profile_push(out);
        }
        continue;
    }

    if (!cc.pushIn(mv))
    {
        handle_comp_error(mv); // input buffer full
        continue;
    }

    if (!cc.process())
    {
        handle_comp_error(mv); // output space / processing failure
        continue;
    }

    Move2D out;
    while (cc.popOut(out))
        profile_push(out);

    if (s.sawG40)
    {
        cc.flush();
        cc.setComp(COMP_OFF);

        while (cc.popOut(out))
            profile_push(out);
    }
}

// end-of-program drain
cc.flush();
Move2D out;
while (cc.popOut(out))
    profile_push(out);
```

## Practical notes for embedded targets

- `interpret_move` owns modal updates (`motionG`, `comp`, `feed`, `pos`, `z`); avoid duplicating that logic elsewhere.
- `G41`/`G42` is applied before processing the current move; motion-line `G40` is applied after processing that move.
- Non-motion `G40` lines are handled via the `mv.type == MOT_EMPTY` branch.
- Keep parser/comp in task context (not ISR); keep profile/planner handoff non-blocking when possible.
