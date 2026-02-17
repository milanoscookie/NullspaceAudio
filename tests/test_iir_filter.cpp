// Tests for IIRFilter (2nd-order biquad)
#include "test_harness.h"
#include "utils/IIRFilter.h"

TEST(identity_passthrough_sample) {
  IIRFilter f(IIRFilter::identityCoeffs());
  // Identity: b0=1, b1=b2=a1=a2=0 => output = input
  ASSERT_NEAR(f.filterSample(1.0f), 1.0f, 1e-7f);
  ASSERT_NEAR(f.filterSample(0.5f), 0.5f, 1e-7f);
  ASSERT_NEAR(f.filterSample(-0.3f), -0.3f, 1e-7f);
}

TEST(identity_passthrough_block) {
  IIRFilter f;
  IIRFilter::Block input = IIRFilter::Block::Random();
  const IIRFilter::Block &output = f.filterBlock(input);
  for (int i = 0; i < dsp::BLOCK_SIZE; ++i) {
    ASSERT_NEAR(output(i), input(i), 1e-6f);
  }
}

TEST(zero_coeffs_produce_zero) {
  IIRFilter::FilterCoeff c;
  c << 0.0f, 0.0f, 0.0f, 0.0f, 0.0f;
  IIRFilter f(c);
  ASSERT_NEAR(f.filterSample(1.0f), 0.0f, 1e-7f);
  ASSERT_NEAR(f.filterSample(1.0f), 0.0f, 1e-7f);
}

TEST(pure_delay_one_sample) {
  // b0=0, b1=1, b2=0, a1=0, a2=0 => y[n] = x[n-1]
  IIRFilter::FilterCoeff c;
  c << 0.0f, 1.0f, 0.0f, 0.0f, 0.0f;
  IIRFilter f(c);
  ASSERT_NEAR(f.filterSample(5.0f), 0.0f, 1e-7f);  // x[-1] = 0
  ASSERT_NEAR(f.filterSample(3.0f), 5.0f, 1e-7f);  // x[0] = 5
  ASSERT_NEAR(f.filterSample(1.0f), 3.0f, 1e-7f);  // x[1] = 3
}

TEST(feedback_accumulator) {
  // b0=1, a1=-1 => y[n] = x[n] + y[n-1] (integrator)
  IIRFilter::FilterCoeff c;
  c << 1.0f, 0.0f, 0.0f, -1.0f, 0.0f;
  IIRFilter f(c);
  ASSERT_NEAR(f.filterSample(1.0f), 1.0f, 1e-6f);
  ASSERT_NEAR(f.filterSample(1.0f), 2.0f, 1e-6f);
  ASSERT_NEAR(f.filterSample(1.0f), 3.0f, 1e-6f);
}

TEST(setCoefficients) {
  IIRFilter f;
  IIRFilter::FilterCoeff c;
  c << 0.5f, 0.0f, 0.0f, 0.0f, 0.0f;
  f.setCoefficients(c);
  ASSERT_NEAR(f.filterSample(2.0f), 1.0f, 1e-7f);
}

TEST(dc_gain) {
  // For a biquad, DC gain = (b0+b1+b2) / (1+a1+a2)
  // Pick b0=0.2, b1=0.3, b2=0.1, a1=0.1, a2=0.05
  // DC gain = 0.6 / 1.15 ≈ 0.5217
  IIRFilter::FilterCoeff c;
  c << 0.2f, 0.3f, 0.1f, 0.1f, 0.05f;
  IIRFilter f(c);
  // Feed DC (1.0) for many samples, should converge to DC gain
  float y = 0.0f;
  for (int i = 0; i < 1000; ++i)
    y = f.filterSample(1.0f);
  float expected_dc = (0.2f + 0.3f + 0.1f) / (1.0f + 0.1f + 0.05f);
  ASSERT_NEAR(y, expected_dc, 1e-4f);
}

int main() {
  RUN_TEST(identity_passthrough_sample);
  RUN_TEST(identity_passthrough_block);
  RUN_TEST(zero_coeffs_produce_zero);
  RUN_TEST(pure_delay_one_sample);
  RUN_TEST(feedback_accumulator);
  RUN_TEST(setCoefficients);
  RUN_TEST(dc_gain);
  PRINT_RESULTS();
  return g_fails > 0 ? 1 : 0;
}
