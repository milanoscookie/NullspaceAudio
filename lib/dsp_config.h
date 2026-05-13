#pragma once

#include <numbers>

#include <Eigen/Dense>

#ifndef EIGEN_FFTW_DEFAULT
#define EIGEN_FFTW_DEFAULT
#endif

#include <unsupported/Eigen/FFT>

namespace dsp {

constexpr int SAMPLE_RATE = 48000;

// Block size: 256 samples @ 48kHz = ~5.3ms latency per block
constexpr int BLOCK_SIZE = 256;

// Impulse response size for convolution (1024 @ 48kHz = ~21ms)
constexpr int IR_SIZE = 4096;

// IIR filter state size (for biquad: 2)
constexpr int IIR_STATE_SIZE = 2;

// Number of blocks to buffer for latency compensation
constexpr int CONTEXT_BLOCKS = IR_SIZE / BLOCK_SIZE; // 4 blocks for 1024-sample IR

constexpr int BLOCK_LATENCY_US =
    (BLOCK_SIZE * 1'000'000) / SAMPLE_RATE; // ~5333us

using Block = Eigen::Matrix<float, BLOCK_SIZE, 1>;
using IRBlock = Eigen::Matrix<float, IR_SIZE, 1>;
using IIRState = Eigen::Matrix<float, IIR_STATE_SIZE, 1>;

namespace sim {

struct ResonanceConfig {
  bool enabled = false;
  float frequencyHz = 0.0f;
  float q = 1.0f;
  float gainDb = 0.0f;
};

struct PathConfig {
  int delaySamples = 0;
  float gain = 1.0f;
  float lowpassHz = 0.0f;
  ResonanceConfig resonance;
};

struct NoiseSweepConfig {
  float centerFreqMinHz = 300.0f;
  float centerFreqMaxHz = 700.0f;
  float sweepHalfPeriodSeconds = 0.5f;
  float bandwidthHz = 30.0f;
  float centerFreqBrownianStddevHz = 3.0f;
  float centerFreqBrownianPole = 0.99f;
  float sampleSigma = 0.01f;
  float noiseGain = 4.0f;
};

struct ModelConfig {
  PathConfig reference;
  PathConfig primary;
  PathConfig secondary;
  PathConfig feedback;
  NoiseSweepConfig noise;
  float secondaryPathDriftGain = 0.001f;
  float wavToReferenceGain = 0.0f;
  bool enableReferenceFeedback = true;
};

inline constexpr ResonanceConfig kNoResonance{};

inline constexpr ResonanceConfig kPrimaryResonance{
    .enabled = true,
    .frequencyHz = 500.0f,
    .q = 1.2f,
    .gainDb = 1.5f,
};

inline constexpr ResonanceConfig kSecondaryResonance{
    .enabled = true,
    .frequencyHz = 3000.0f,
    .q = 2.0f,
    .gainDb = 5.0f,
};

inline constexpr PathConfig kReferencePath{
    .delaySamples = 2,
    .gain = 1.0f,
    .lowpassHz = 8000.0f,
    .resonance = kNoResonance,
};

inline constexpr PathConfig kPrimaryPath{
    .delaySamples = 32,
    .gain = 0.7f,
    .lowpassHz = 1500.0f,
    .resonance = kPrimaryResonance,
};

inline constexpr PathConfig kSecondaryPath{
    .delaySamples = 16,
    .gain = 1.0f,
    .lowpassHz = 4000.0f,
    .resonance = kSecondaryResonance,
};

inline constexpr PathConfig kFeedbackPath{
    .delaySamples = 8,
    .gain = 0.10f,
    .lowpassHz = 3000.0f,
    .resonance = kNoResonance,
};

inline constexpr NoiseSweepConfig kNoiseSweep{};

inline constexpr ModelConfig kDefaultModelConfig{
    .reference = kReferencePath,
    .primary = kPrimaryPath,
    .secondary = kSecondaryPath,
    .feedback = kFeedbackPath,
    .noise = kNoiseSweep,
    .secondaryPathDriftGain = 1.0e-7f,
    .wavToReferenceGain = 0.1f,
    .enableReferenceFeedback = true,
};

constexpr float kTwoPi = 2.0f * std::numbers::pi_v<float>;

constexpr float samplesToSeconds(int samples) {
  return static_cast<float>(samples) / static_cast<float>(SAMPLE_RATE);
}

} // namespace sim

} // namespace dsp
