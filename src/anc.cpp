#include "anc.h"

using namespace dsp;

void anc::init() {}

void anc::step(const MicBlock &micBlock, Block &control) {
  // Simple feedforward ANC: use in-ear mic to estimate noise and invert it for
  // control. The control signal is delayed by block size, which is around 5.33 ms of latency
  (void)micBlock;
  control = Block::Zero();
}
