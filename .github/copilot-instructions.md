# Project instructions for Copilot

## Stack and scope
- This repository is C++ geometry/toolpath code with CMake and host-side simulation.
- Prefer minimal, surgical changes in existing files over broad refactors.
- Do not introduce new dependencies unless explicitly requested.
- Avoid lambda functions unless necessary; prefer named functions for clarity and testability.

## Coding style
- Match existing formatting and naming in touched files.
- Keep math and geometry helpers consistent with existing `Vec2`/`Move2D` patterns.
- Avoid one-letter variable names except for short-lived geometric conventions (`a0`, `a1`, etc.).
- Do not add inline comments unless asked; keep existing comments when useful.

## Behavior and compatibility
- Preserve current behavior unless the task explicitly asks for behavior changes.
- Keep public data structures and function signatures stable unless requested.
- For cleanup work, remove duplication but keep logic/results equivalent.

## Debug and conditional code
- Prefer `#ifndef NDEBUG` or `#if defined(DEBUG) || defined(_DEBUG)` over `#if DEBUG`.
- Keep debug-only fields and diagnostics guarded behind compile-time checks.

## Validation workflow
- After edits, check errors on changed files first.
- For build requests in this CMake workspace, use VS Code CMake build tools/tasks.
- If tests are available for touched logic, run focused tests before broader runs.

## Non-goals
- Do not fix unrelated issues or reformat unrelated code.
- Do not create commits or branches unless explicitly asked.
