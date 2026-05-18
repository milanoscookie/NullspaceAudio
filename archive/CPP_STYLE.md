# C++ Style Guide


All callback-reachable public functions must be explicitly documented as RT-safe.
All functions that allocate, block, log, or throw must be documented as RT-unsafe.
RT-safe code must not call RT-unsafe code.


## Purpose

This project is a C++23 DSP codebase with real-time constraints. The goal of this guide is to keep code readable, predictable, portable, and safe for low-latency audio processing.

There are two standards in this repo:

- general C++ code should be clear, consistent, and easy to maintain
- real-time DSP code must also be deterministic, non-blocking, and allocation-free on the hot path

## Core Principles

- Prefer simple code over clever code.
- Keep reusable DSP and audio infrastructure in `lib/`.
- Keep application orchestration and CLI logic in `src/`.
- Keep tests in `tests/` with focused coverage per component.
- Treat any function reachable from the audio callback or DSP processing loop as real-time constrained.

## Language And Build

- Use C++23.
- Use `#pragma once` in headers.
- Prefer RAII for ownership and cleanup.
- Prefer target-based CMake configuration.
- Keep compiler warnings enabled and fix new warnings promptly.

## Formatting

- Use 2-space indentation.
- Do not use tabs.
- Use K&R brace style.
- Keep one declaration per line unless grouping is clearer and trivial.
- Prefer `const Type &value` style for references.
- Prefer `\n` over `std::endl` unless an explicit flush is required.
- Keep functions short when practical, especially on callback-reachable paths.

## Naming

- Types, classes, structs, and enums use `PascalCase`.
- Functions, methods, local variables, and parameters use `lowerCamelCase`.
- Private data members use `lowerCamelCase_`.
- New public struct fields should use `lowerCamelCase`.
- New constants should use `kCamelCase`.
- Existing compile-time DSP constants in `lib/dsp_config.h` may remain `ALL_CAPS`.
- New file names should use `snake_case`.

## File Organization

- Put reusable DSP, audio, utility, and synchronization components in `lib/`.
- Put executable-specific glue code in `src/`.
- Prefer one main class or component per file.
- Keep headers focused on interface and source files focused on implementation.

## Headers, Includes, And Namespaces

- Order includes as:
  - own header
  - project headers
  - third-party headers
  - standard library headers
- Keep public headers as lightweight as possible.
- Prefer forward declarations when they reduce header coupling safely.
- Avoid introducing new global symbols in public headers.
- Prefer project namespaces for library code.
- Avoid duplicating shared type aliases across multiple public headers.

## API And Ownership Rules

- Make ownership explicit.
- Prefer `std::unique_ptr` for exclusive ownership.
- Use references or pointers only when ownership is not transferred.
- Pass large objects by `const &`.
- Write results into caller-provided outputs when performance matters.
- Prefer fixed-size data types and compile-time shapes for DSP blocks.

## Comments And Documentation

- Write comments for intent, invariants, timing assumptions, and DSP reasoning.
- Do not comment obvious syntax or restate the code line-by-line.
- Remove stale commented-out code instead of keeping it in place.
- Document thread-safety and real-time expectations at API boundaries.

## Real-Time DSP Rules

Any function reachable from the audio callback, block-processing callback, or fast DSP loop must be real-time safe.

Real-time safe means:

- no dynamic allocation
- no blocking
- no file I/O
- no console logging
- no exceptions
- no unbounded work
- no locks with contention risk
- no waiting on other threads

## Real-Time Boundary Annotations

C++ has no standard `[[realtime_safe]]`, `[[hot_path]]`, or `[[no_alloc]]`
attribute. This project uses explicit documentation labels instead.

Use:

- `RT-safe` for functions that may be called from the audio callback,
  block-processing callback, or fast DSP loop.
- `RT-unsafe` for setup, teardown, UI, file I/O, logging, allocation,
  or control-plane functions.

RT-safe functions must not directly or indirectly call RT-unsafe functions.

RT-safe functions should be marked `noexcept` whenever accurate.

Example:

```cpp
// RT-safe.
// Processes one fixed-size block using preallocated state.
// Does not allocate, block, throw, log, perform I/O, or perform unbounded work.
void processBlock(std::span<const float> input,
                  std::span<float> output) noexcept;

// RT-unsafe.
// Allocates buffers and computes coefficients.
// Must be called before streaming starts.
void prepare(const ProcessorConfig& config);
```

### Hot Path Requirements

- Preallocate all buffers, filter state, scratch storage, and workspaces before streaming starts.
- Reuse long-lived storage across blocks.
- Prefer fixed-size Eigen types and fixed-capacity buffers.
- Keep per-block work deterministic and bounded.
- Separate setup work from processing work.
- If code may run on the hot path, design it as if every microsecond matters.

### Forbidden On The Hot Path

- `new`, `delete`, `make_unique`, `make_shared`
- growing `std::vector`, `std::string`, or other heap-backed containers
- `push_back`, `insert`, or any operation that may reallocate unless capacity is fixed and guaranteed
- file writes, file reads, or path inspection
- `std::cout`, `std::cerr`, logging frameworks, or debug printing
- `sleep_for`, `sleep_until`, `join`, `future::wait`, `condition_variable::wait`
- blocking mutex usage
- launching threads per callback or per block
- exception throwing or exception-heavy control flow

### Preferred Patterns For Real-Time Code

- fixed-size block processing
- stack allocation for small temporary values
- preallocated member scratch buffers
- SPSC ring buffers and double buffers for thread handoff
- copy-latest or consume-if-available semantics
- safe fallback outputs such as zero control when deadlines are missed

### Deadline Behavior

- Deadline handling must be explicit and predictable.
- If computation misses the budget, fall back to a documented safe output.
- Fallback behavior must not itself block.
- Prefer silence or zero-control output over stale or partially written state unless the algorithm explicitly requires hold-last-value behavior.

### Eigen Rules

- Prefer fixed-size Eigen types on the hot path when dimensions are known.
- Avoid dynamic-size Eigen allocation in callback-reachable code.
- Use `Eigen::Map` to operate on caller-owned buffers.
- Be explicit about row-major vs column-major layout where memory access matters.
- Watch for hidden temporaries in chained expressions.
- Use `.noalias()` when assigning matrix products where aliasing is impossible.
- Do not call APIs that resize Eigen objects on the hot path.
- Preallocate all dynamic Eigen objects before streaming starts.
### Floating-Point Rules

- Prefer `float` for real-time audio sample processing unless precision requirements justify `double`.
- Use `double` for coefficient design, offline analysis, accumulators, or numerically sensitive setup-time calculations when needed.
- Be explicit when converting between `float` and `double`.
- Real-time DSP code should avoid denormal slowdowns.
- Enable flush-to-zero / denormals-are-zero where supported, or use a project-approved denormal guard.
- Do not enable unsafe floating-point compiler flags globally unless the numerical consequences are understood and documented.

### Data Layout

- Prefer structure-of-arrays for large numeric streams and vectorized processing.
- Prefer array-of-structures for small control/configuration objects where locality across fields matters.
- Document channel/sample layout at API boundaries.
- Avoid implicit layout assumptions.

### Units And Signal Conventions

- Names should include physical units where ambiguity is likely:
  - `sampleRateHz`
  - `cutoffHz`
  - `durationSeconds`
  - `phaseRadians`
  - `delaySamples`
  - `velocityMps`
- Do not mix samples, seconds, radians, degrees, bins, and Hz without explicit conversion.
- Document sign conventions for phase, Doppler, FFT shifts, and coordinate systems.
- Prefer small typed wrappers or strongly named structs for high-risk quantities.

### FFT And Spectral Processing

- Every FFT-using component must document:
  - transform direction convention
  - normalization convention
  - bin-to-frequency mapping
  - whether spectra are shifted or unshifted
  - real vs complex layout
  - window function used
  - overlap/hop size where applicable

- Do not assume all FFT libraries use the same scaling.
- Wrap third-party FFT calls behind project-owned interfaces.

## Dependencies

- Third-party dependencies must be added through target-based CMake.
- Pin dependency versions where reproducibility matters.
- Wrap third-party APIs at project boundaries when practical.
- Do not expose large third-party types in public headers unless they are part of the deliberate public API.
- Avoid adding dependencies to hot-path code unless their allocation and threading behavior is understood.


  ### Const, constexpr, noexcept, And Attributes

- Use `const` for local variables when it clarifies immutability.
- Do not use `const` mechanically when it reduces readability without improving safety.
- Use `constexpr` for true compile-time constants and small pure functions that are meaningfully usable at compile time.
- Use `consteval` only when compile-time evaluation is required.
- Use `[[nodiscard]]` for functions where ignoring the result is likely a bug:
  - status-returning functions
  - allocation/setup factories
  - validation functions
  - functions returning computed DSP metadata
- Avoid `[[nodiscard]]` on trivial getters unless ignoring the result is genuinely suspicious.
- Use `noexcept` on destructors, move operations, and RT-safe functions where accurate.

### Atomics

- Prefer simple ownership transfer over shared mutable state.
- Prefer SPSC queues or double buffers for audio/control communication.
- Use `std::atomic` for scalar cross-thread state.
- Default to `memory_order_seq_cst` unless profiling or design requires weaker ordering.
- If using acquire/release/relaxed ordering, document why it is correct.
- Do not use `std::shared_ptr` handoff on the audio thread unless its refcounting behavior is proven acceptable.
### SIMD And Vectorization

- Prefer clear scalar code first unless profiling shows the block is hot.
- Use Eigen vectorization, `std::simd`, or explicit intrinsics only where justified by measurement.
- Keep SIMD code isolated behind small functions or implementation files.
- Provide a scalar reference path for testing when practical.
- Do not sacrifice numerical correctness or phase conventions for micro-optimizations without documenting the tradeoff.
### Concurrency Rules

- The audio thread must never wait for another thread.
- Background threads may publish data; the DSP path should consume only what is immediately available.
- Shared state must have a documented ownership and update model.
- Prefer single-producer/single-consumer structures where they fit the data flow.
- Avoid detached threads unless there is a clearly documented lifecycle reason.
- If synchronization is required off the hot path, keep the locked region small and obvious.

### Performance And Memory Rules

- Optimize for worst-case block latency, not only average runtime.
- Avoid hidden heap traffic.
- Avoid hidden temporaries in math-heavy code where practical.
- Minimize copies and conversions.
- Keep memory layout simple and predictable.
- Reserve capacity before use in non-real-time code if repeated appends are expected.
- Measure before and after optimization changes.

### Error Handling

- Use exceptions for setup-time failures when they simplify API use.
- Do not throw from callback-reachable code.
- Use explicit fallback behavior or status reporting in runtime processing paths.
- Fail early during initialization if required resources are missing.

## Tooling And Enforcement

- Code must pass `clang-format`.
- Code must pass configured compiler warnings.
- New code should pass `clang-tidy` checks used by the project.
- CI should build at least:
  - Debug
  - Release
  - sanitizer configuration
  - tests
  - benchmarks where practical
## Testing And Benchmarking

- Keep one test file per component where practical.
- Use descriptive `snake_case` test names.
- Separate functional tests from timing or stress tests.
- Avoid hardcoded absolute paths in tests.
- Use temporary directories portably.
- Add tests for real-time behavior when relevant, including timeout fallback and shutdown behavior.
- Benchmark worst-case block processing time for hot-path changes.

## Do And Don't Examples

Do:

- precompute filter coefficients before processing starts
- reuse Eigen blocks and scratch buffers
- buffer diagnostic or output data in memory and write it after processing ends
- use ring buffers or double buffers for cross-thread handoff

Don't:

- write WAV files inside the processing callback
- allocate vectors or strings inside the DSP loop
- spawn worker threads per block
- log timeout messages from a callback at high frequency
- depend on another thread completing before the audio thread can continue

## Callback-Safe Checklist

Before merging any callback-reachable code, confirm all of the following:

- No allocation happens on the hot path.
- No file I/O or console output happens on the hot path.
- No blocking synchronization or waiting happens on the hot path.
- Work per block is bounded and deterministic.
- All buffers and state are preallocated.
- Fallback behavior on missed deadlines is documented and tested.
- Thread ownership and data handoff are clear.
- Functional tests and any relevant timing tests pass.

## Scope For Existing Code

This guide applies immediately to new and modified code. Existing code does not need wholesale rewrites, but changes should move the codebase toward these rules over time.
