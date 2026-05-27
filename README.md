# Nullspace ANC Simulator

Welcome to the Nullspace Active Noise Cancellation project.

In this project, you will implement a real-time ANC controller for simulated airplane engine noise. The full technical explanation, acoustic plant model, timing constraints, grading rubric, and implementation requirements are provided in:


[docs/problem_statement.pdf](docs/problem_statement.pdf)


Read that PDF before writing your controller. This README is only meant to help you get the repository set up and run the simulator.

---

## 1. Fork This Repository

You should work from your own fork of this repository.

1. Open this repository on GitHub.
2. Click **Fork** in the top-right corner.
3. Select your GitHub account as the owner.
4. Click **Create fork**.
5. Clone your fork locally:

```bash
git clone https://github.com/YOUR-USERNAME/YOUR-FORKED-REPO-NAME.git
cd YOUR-FORKED-REPO-NAME
```

---

## 2. Install Dependencies

This project uses CMake and a C++ build toolchain. The main dependencies are Eigen for Linear Algebra and FFTW for DFTs.

### Linux / WSL


```bash
sudo apt update
sudo apt install build-essential cmake libfftw3-dev
```

> Windows users make sure to download WSL. Docs are in [docs/problem_statement.pdf](docs/problem_statement.pdf)

If you want to profile your code with `perf`, also install:

```bash
sudo apt install linux-tools-generic linux-tools-common
```

### macOS

Install Apple’s command-line tools:

```bash
xcode-select --install
```

Install CMake and FFTW with Homebrew:

```bash
brew update
brew install cmake fftw
```

---

## 3. Build the Simulator

From the repository root:

```bash

mkdir build
cd build
cmake ..
make
```

If the build succeeds, you are ready to run the simulator.

---

## 4. Run a First Simulation

Run the simulator on the provided input file:

```bash
./build/src/DSPApp src/input.wav trial1
```

This will generate output WAV files such as:

```text
trial1_raw_input.wav
trial1_outside_mic.wav
trial1_inear_mic.wav
```

You can inspect these files in Audacity, Python, or another audio analysis tool.

---

## 5. Where You Write Code

Your main controller implementation goes in:

```text
src/anc.cpp
```

and its corrosponding headerfile:

```text
include/anc.h
```

The simulator calls your controller through:

```cpp
void anc::step(const MicBlock& micBlock, dsp::Block& control);
```

Each call gives you one block of microphone data. Your job is to fill `control` with the anti-noise signal that should be played through the speaker.

---

## 6. Important Timing Constraint

The simulator runs at:

```text
Sample rate: 48,000 Hz
Block size:  256 samples
Block time:  5.33 ms
```

Your controller must finish each call to `anc::step(...)` within the block deadline.

Avoid real-time unsafe operations inside the callback, including:

- heap allocations
- file I/O
- `std::cout` or other printing
- sleeping
- blocking locks
- dynamically growing containers

The simulator also introduces a mandatory one-block transport delay before your control block reaches the speaker. This delay is one of the central challenges of the project.

---

## 7. Read the Full Project Statement

Before implementing anything serious, read:

```text
docs/problem_statement.pdf
```

The PDF explains:

- the ANC objective
- the microphone and speaker signal paths
- the `MicBlock` and `dsp::Block` data structures
- the 5.33 ms real-time deadline
- the acoustic paths `H`, `P`, `S`, and `C`
- offline Python prototyping suggestions
- profiling guidance
- report and grading requirements

The PDF is the source of truth for the assignment.

---

## 8. Submission

Submit a link to your GitHub repository containing:

- your `anc.cpp` implementation
- any supporting source files
- Python scripts and plots used for analysis
- your final technical report

See `docs/problem_statement.pdf` for the full rubric and report requirements.
