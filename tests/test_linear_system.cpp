// Tests for LinearSystem<IR_SIZE> (block-based FIR convolution)
#include "test_harness.h"
#include "utils/LinearSystem.h"

using LS = LinearSystem<dsp::IR_SIZE>;
using Block = LS::Block;
using IRBlock = LS::IRBlock;

TEST(zero_ir_produces_zero) {
  LS sys;
  Block input = Block::Ones();
  Block output;
  sys.step(input, output);
  for (int i = 0; i < dsp::BLOCK_SIZE; ++i) {
    ASSERT_NEAR(output(i), 0.0f, 1e-7f);
  }
}

TEST(delta_ir_is_passthrough) {
  // h = [1, 0, 0, ...] => output = input
  IRBlock h = IRBlock::Zero();
  h(0) = 1.0f;
  LS sys(h);
  Block input;
  for (int i = 0; i < dsp::BLOCK_SIZE; ++i)
    input(i) = static_cast<float>(i + 1);

  Block output;
  sys.step(input, output);

  for (int i = 0; i < dsp::BLOCK_SIZE; ++i) {
    ASSERT_NEAR(output(i), input(i), 1e-5f);
  }
}

TEST(single_delay) {
  // h = [0, 1, 0, ...] => output[n] = input[n-1]
  IRBlock h = IRBlock::Zero();
  h(1) = 1.0f;
  LS sys(h);

  Block input1 = Block::Zero();
  input1(dsp::BLOCK_SIZE - 1) = 7.0f; // last sample of block 1

  Block output1;
  sys.step(input1, output1);
  // output1[0] should be 0 (no previous sample)
  ASSERT_NEAR(output1(0), 0.0f, 1e-7f);
  // output1[BLOCK_SIZE-1] should be input1[BLOCK_SIZE-2]
  if (dsp::BLOCK_SIZE > 1) {
    ASSERT_NEAR(output1(dsp::BLOCK_SIZE - 1), 0.0f, 1e-7f);
  }

  // Second block — first sample should be last sample of block 1
  Block input2 = Block::Zero();
  Block output2;
  sys.step(input2, output2);
  ASSERT_NEAR(output2(0), 7.0f, 1e-5f);
}

TEST(scaling) {
  // h = [3, 0, 0, ...] => output = 3 * input
  IRBlock h = IRBlock::Zero();
  h(0) = 3.0f;
  LS sys(h);
  Block input = Block::Ones();
  Block output;
  sys.step(input, output);
  for (int i = 0; i < dsp::BLOCK_SIZE; ++i) {
    ASSERT_NEAR(output(i), 3.0f, 1e-5f);
  }
}

TEST(impulse_response_recovery) {
  // Feed a single-sample impulse, collect IR_SIZE output samples
  IRBlock h = IRBlock::Random();
  LS sys(h);

  // Block 1: impulse at sample 0
  Block input = Block::Zero();
  input(0) = 1.0f;
  Block output;
  sys.step(input, output);

  // First BLOCK_SIZE samples of impulse response
  for (int i = 0; i < dsp::BLOCK_SIZE && i < dsp::IR_SIZE; ++i) {
    ASSERT_NEAR(output(i), h(i), 1e-5f);
  }
}

TEST(setImpulseResponse) {
  LS sys;
  IRBlock h = IRBlock::Zero();
  h(0) = 2.0f;
  sys.setImpulseResponse(h);
  Block input = Block::Ones();
  Block output;
  sys.step(input, output);
  ASSERT_NEAR(output(0), 2.0f, 1e-5f);
}

TEST(linearity) {
  // f(a*x) = a*f(x) for FIR
  IRBlock h = IRBlock::Random();
  LS sys1(h);
  LS sys2(h);

  Block x = Block::Random();
  float a = 2.5f;

  Block y1, y2;
  sys1.step(x, y1);
  sys2.step(a * x, y2);

  for (int i = 0; i < dsp::BLOCK_SIZE; ++i) {
    ASSERT_NEAR(y2(i), a * y1(i), 1e-3f);
  }
}

int main() {
  RUN_TEST(zero_ir_produces_zero);
  RUN_TEST(delta_ir_is_passthrough);
  RUN_TEST(single_delay);
  RUN_TEST(scaling);
  RUN_TEST(impulse_response_recovery);
  RUN_TEST(setImpulseResponse);
  RUN_TEST(linearity);
  PRINT_RESULTS();
  return g_fails > 0 ? 1 : 0;
}
