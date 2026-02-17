#pragma once

#include <cstddef>

#include <Eigen/Dense>
#include <unsupported/Eigen/FFT>

namespace dsp {

constexpr int SAMPLE_RATE = 48000;

// Block size: 256 samples @ 48kHz = ~5.3ms latency per block
constexpr size_t BLOCK_SIZE = 256;

// Impulse response size for convolution (256 @ 48kHz = ~5.3ms)
constexpr size_t IR_SIZE = 256;

// IIR filter state size (for biquad: 2)
constexpr size_t IIR_STATE_SIZE = 2;

// Number of blocks to buffer for latency compensation
constexpr size_t CONTEXT_BLOCKS = 4;

constexpr float BLOCK_LATENCY_MS =
    ((BLOCK_SIZE * 1000.0f) / static_cast<float>(SAMPLE_RATE)); // ~5ms

} // namespace dsp
