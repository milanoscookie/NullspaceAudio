#include "dsp_interface.h"

namespace anc {

constexpr int systemLatencyBlocks = 1;
void init();
void step(const MicBlock &micBlock, dsp::Block &control);

} // namespace anc
