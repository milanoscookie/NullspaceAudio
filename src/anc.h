#include "dsp_interface.h"

namespace anc {

constexpr int systemLatencyBlocks = 4;
void step(const MicBlock &micBlock, Block &control);
} // namespace anc