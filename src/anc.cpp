#include "anc.h"
void anc::step(const MicBlock &micBlock, Block &control) {
    // Simple feedforward ANC: use in-ear mic to estimate noise and invert it for control. This will work work due to acoustic noise propigation
    // The control signal is delayed by systemLatencyBlocks * block size samples to account for the time it takes for the control signal to propagate through the system and for the next mic block to be read in. 
    control = -micBlock.inear;
}