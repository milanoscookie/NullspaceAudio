#pragma once

#include "dsp_config.h"

namespace dsp::sim {

struct PathSet {
  IRBlock reference = IRBlock::Zero();
  IRBlock primary = IRBlock::Zero();
  IRBlock secondary = IRBlock::Zero();
  IRBlock feedback = IRBlock::Zero();
  IRBlock speaker = IRBlock::Zero();
};

// RT-unsafe.
// Synthesizes physics-informed FIR path models for the simulator.
[[nodiscard]] PathSet buildPathSet(
    const ModelConfig &config = kDefaultModelConfig);

} // namespace dsp::sim
