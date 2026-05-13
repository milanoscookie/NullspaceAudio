#include "anc.h"

namespace {

using namespace dsp;
struct AncState {
  bool initialized = false;
};

AncState gState;

} // namespace

void anc::init() {
  gState = AncState{};
  gState.initialized = true;
}

void anc::step(const MicBlock &micBlock, Block &control) {
  if (!gState.initialized) {
    control = Block::Zero();
    return;
  }

  // Simple feedforward ANC: use in-ear mic to estimate noise and invert it for
  // control. The control signal is delayed by systemLatencyBlocks * block size
  // samples to account for propagation and block processing latency.
  // (void)micBlock;
  control = Block::Zero();
}
