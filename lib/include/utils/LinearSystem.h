#pragma once

#include "RingBuffer.h"
#include "dsp_config.h"
#include <Eigen/Dense>

template <int IR_SIZE> class LinearSystem {
public:
  using Block = dsp::Block;
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

  void step(const Block &input, Block &output) {
    inputHistory_.push_back(input);
    output.setZero();

    for (int n = 0; n < dsp::BLOCK_SIZE; ++n) {
      float y = 0.0f;

      for (int k = 0; k < IR_SIZE; ++k) {
        const int x_index = n - k;

        if (x_index >= 0) {
          y += impulseResponse_(k) * input(x_index);
        } else {
          const int past = (-x_index - 1) / dsp::BLOCK_SIZE +
                           1;
          const int idx_in_block =
              x_index + past * dsp::BLOCK_SIZE;

          if (past <= inputHistory_.size()) {
            const Block *b = inputHistory_.from_back(past);
            if (b) {
              y += impulseResponse_(k) * (*b)(idx_in_block);
            }
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
