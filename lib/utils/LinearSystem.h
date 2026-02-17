#pragma once

#include "RingBuffer.h"
#include "dsp_config.h"
#include <Eigen/Dense>

template <int IR_SIZE> class LinearSystem {
public:
  using Block = Eigen::Matrix<float, dsp::BLOCK_SIZE, 1>;
  using IRBlock = Eigen::Matrix<float, IR_SIZE, 1>;

    static constexpr int kNumBlocks =
      (IR_SIZE + dsp::BLOCK_SIZE - 1) / dsp::BLOCK_SIZE + 1;

  using InputHistoryBuffer = RingBuffer<Block, kNumBlocks>;

  LinearSystem() = default;
  LinearSystem(const IRBlock &impulseResponse)
      : impulseResponse_(impulseResponse) {}

  void setImpulseResponse(const IRBlock &impulseResponse) {
    impulseResponse_ = impulseResponse;
  }

  const IRBlock &getImpulseResponse() const { return impulseResponse_; }

  // Block-based FIR: output = sum_{k=0}^{IR_SIZE-1} h[k] * x[n-k]
  // contiguous block processing with a history of past blocks.
  void step(const Block &input, Block &output) {
    inputHistory_.push_back(input);
    output.setZero();

    // For each output sample within the block
    for (int n = 0; n < dsp::BLOCK_SIZE; ++n) {
      float y = 0.0f;

      // Convolve with h[0..IR_SIZE-1]
      for (int k = 0; k < IR_SIZE; ++k) {
        const int x_index = n - k; // relative to current block start

        if (x_index >= 0) {
          // current block
          y += impulseResponse_(k) * input(x_index);
        } else {
          // previous blocks
          const int past = (-x_index - 1) / dsp::BLOCK_SIZE +
                           1; // 1 => immediately previous block
          const int idx_in_block =
              x_index + past * dsp::BLOCK_SIZE; // bring into [0, BLOCK_SIZE)

          if (past <= inputHistory_.size()) {
            const Block *b = inputHistory_.from_back(past);
            if (b) {
              y += impulseResponse_(k) * (*b)(idx_in_block);
            }
          } else {
            // this is older than history buffer, treat as zero
          }
        }
      }

      output(n) = y;
    }
  }

private:
  IRBlock impulseResponse_ = IRBlock::Zero();
  InputHistoryBuffer inputHistory_;
};
