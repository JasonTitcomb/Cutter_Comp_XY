# Embedded Cutter Compensation Flow

This document captures the recommended embedded runtime flow for `CutterComp2D` based on the current host-side behavior.

## High-level flow

- Initialize parser modal state.
- Configure `CutterComp2D` (`setToolRadius`, `setMachineType`, `setCornerTreatment`, `setComp(COMP_OFF)`).
- For each parsed G-code line:
  - Interpret line to `Move2D`.
  - If line enables comp (`G41`/`G42`), call `setComp(...)` before processing the move.
  - Push input with `pushIn` (handle backpressure if full).
  - Call `process` until successful or unrecoverable failure policy triggers.
  - Drain all available output via `popOut` and send to motion queue/planner.
  - On `G40`, call `flush`, drain output, then `setComp(COMP_OFF)`.
- At program end, call final `flush` and drain `popOut`.

## Firmware-style pseudocode

```cpp
CutterComp2D cc;
init_parser_modal();

cc.setMachineType(MAC_MILL);
cc.setToolRadius(toolRadius);
cc.setCornerTreatment(CORNER_ROLL);
cc.setComp(COMP_OFF);

while (read_next_gcode_line(line))
{
    ScanLine s = scan_line(line);
    Move2D mv = interpret_to_move(s, modal, MACHINE_TYPE);

    if (s.sawG41 || s.sawG42)
        cc.setComp(modal.comp);   // set before processing current move

    // backpressure-safe push
    while (!cc.pushIn(mv))
    {
        Move2D out;
        if (cc.popOut(out))
            emit_to_motion_queue(out);
        else
            wait_or_yield();
    }

    // pump until stable (or hard geometry failure policy)
    while (!cc.process())
    {
        Move2D out;
        bool drained = false;
        while (cc.popOut(out))
        {
            emit_to_motion_queue(out);
            drained = true;
        }

        if (!drained)
        {
            // likely non-buffer failure (geometry/invalid offset)
            handle_comp_error(mv);   // alarm, fallback, or bypass
            break;
        }
    }

    // normal drain
    Move2D out;
    while (cc.popOut(out))
        emit_to_motion_queue(out);

    if (s.sawG40)
    {
        cc.flush();
        while (cc.popOut(out))
            emit_to_motion_queue(out);
        cc.setComp(COMP_OFF);
    }
}

// end-of-program drain
cc.flush();
Move2D out;
while (cc.popOut(out))
    emit_to_motion_queue(out);
```

## Practical notes for embedded targets

- Drain `popOut` aggressively to avoid output buffer backpressure.
- Treat `process() == false` as backpressure first; escalate only if nothing drains.
- Keep parser/comp in task context (not ISR); make planner queue handoff non-blocking when possible.
