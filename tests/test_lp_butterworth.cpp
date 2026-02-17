// Tests for LPButterworthCoeff
#include "test_harness.h"
#include "utils/LPButterworthCoeff.h"
#include <cmath>

TEST(coefficients_valid) {
  LPButterworthCoeff bw(1000.0f, 48000.0f);
  auto c = bw.getCoefficients();
  // All coefficients should be finite
  for (int i = 0; i < 5; ++i) {
    ASSERT_TRUE(std::isfinite(c(i)));
  }
}

TEST(dc_gain_is_unity) {
  // Butterworth LPF should pass DC unchanged
  // DC gain = (b0+b1+b2) / (1+a1+a2)
  LPButterworthCoeff bw(1000.0f, 48000.0f);
  auto c = bw.getCoefficients();
  float dc = (c(0) + c(1) + c(2)) / (1.0f + c(3) + c(4));
  ASSERT_NEAR(dc, 1.0f, 1e-5f);
}

TEST(nyquist_gain_is_zero) {
  // At Nyquist (z = -1): H(-1) = (b0 - b1 + b2) / (1 - a1 + a2)
  // For a LPF this should be ~0
  LPButterworthCoeff bw(1000.0f, 48000.0f);
  auto c = bw.getCoefficients();
  float nyquist = (c(0) - c(1) + c(2)) / (1.0f - c(3) + c(4));
  ASSERT_TRUE(std::abs(nyquist) < 0.1f); // should be very small
}

TEST(lower_cutoff_stronger_attenuation) {
  // Lower cutoff should attenuate high frequencies more
  LPButterworthCoeff low(100.0f, 48000.0f);
  LPButterworthCoeff high(5000.0f, 48000.0f);

  // Check b0: lower cutoff => smaller b0 (less passband)
  ASSERT_TRUE(low.getCoefficients()(0) < high.getCoefficients()(0));
}

TEST(setCutoffFrequency) {
  LPButterworthCoeff bw(1000.0f, 48000.0f);
  auto c1 = bw.getCoefficients();
  bw.setCutoffFrequency(2000.0f);
  auto c2 = bw.getCoefficients();
  // Coefficients should change
  ASSERT_TRUE(std::abs(c1(0) - c2(0)) > 1e-8f);
}

TEST(identity_when_cutoff_near_nyquist) {
  // Very high cutoff should be nearly allpass
  LPButterworthCoeff bw(23000.0f, 48000.0f);
  auto c = bw.getCoefficients();
  float dc = (c(0) + c(1) + c(2)) / (1.0f + c(3) + c(4));
  ASSERT_NEAR(dc, 1.0f, 1e-4f);
}

TEST(filter_integration) {
  // Feed DC through a Butterworth filter — should converge to 1.0
  LPButterworthCoeff bw(500.0f, 48000.0f);
  IIRFilter f(bw.getCoefficients());
  float y = 0.0f;
  for (int i = 0; i < 5000; ++i)
    y = f.filterSample(1.0f);
  ASSERT_NEAR(y, 1.0f, 1e-3f);
}

int main() {
  RUN_TEST(coefficients_valid);
  RUN_TEST(dc_gain_is_unity);
  RUN_TEST(nyquist_gain_is_zero);
  RUN_TEST(lower_cutoff_stronger_attenuation);
  RUN_TEST(setCutoffFrequency);
  RUN_TEST(identity_when_cutoff_near_nyquist);
  RUN_TEST(filter_integration);
  PRINT_RESULTS();
  return g_fails > 0 ? 1 : 0;
}
