#pragma once

#include <cstddef>

#include <Eigen/Dense>
#include <unsupported/Eigen/FFT>

namespace dsp {

constexpr int SAMPLE_RATE = 48000;

// Block size: 256 samples @ 48kHz = ~5.3ms latency per block
constexpr size_t BLOCK_SIZE = 256;

// Impulse response size for convolution (1024 @ 48kHz = ~21ms)
constexpr size_t IR_SIZE = 1024;

// IIR filter state size (for biquad: 2)
constexpr size_t IIR_STATE_SIZE = 2;

// Number of blocks to buffer for latency compensation
constexpr size_t CONTEXT_BLOCKS = IR_SIZE / BLOCK_SIZE; // 4 blocks for 1024-sample IR

// constexpr float BLOCK_LATENCY_MS =
//     ((BLOCK_SIZE * 1000.0f) / static_cast<float>(SAMPLE_RATE)); // ~5ms

constexpr int BLOCK_LATENCY_US =
    (BLOCK_SIZE * 1'000'000) / SAMPLE_RATE; // ~5333us


} // namespace dsp
