#include "anc.h"
void anc::step(const MicBlock &micBlock, Block &control) {
    // Simple feedforward ANC: use in-ear mic to estimate noise and invert it for control
    control = -micBlock.inear;
}