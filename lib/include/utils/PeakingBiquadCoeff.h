#pragma once

#include "IIRFilter.h"

#include <algorithm>
#include <cmath>

class PeakingBiquadCoeff {
public:
  using FilterCoeff = IIRFilter::FilterCoeff;

  PeakingBiquadCoeff() = default;
  ~PeakingBiquadCoeff() = default;

  PeakingBiquadCoeff(float centerFreqHz, float q, float gainDb,
                     float sampleRateHz)
      : centerFreqHz_(centerFreqHz), q_(q), gainDb_(gainDb),
        sampleRateHz_(sampleRateHz) {
    calculateCoefficients();
  }

  void setCenterFreqHz(float centerFreqHz) {
    centerFreqHz_ = centerFreqHz;
    calculateCoefficients();
  }

  void setQ(float q) {
    q_ = q;
    calculateCoefficients();
  }

  void setGainDb(float gainDb) {
    gainDb_ = gainDb;
    calculateCoefficients();
  }

  void setSampleRateHz(float sampleRateHz) {
    sampleRateHz_ = sampleRateHz;
    calculateCoefficients();
  }

  const FilterCoeff &getCoefficients() const { return coeffs_; }

  void calculateCoefficients() {
    constexpr float kPi = 3.14159265358979323846f;
    constexpr float kMinSampleRateHz = 1.0f;
    constexpr float kMinQ = 1e-3f;
    constexpr float kNyquistMarginHz = 1.0f;

    const float safeSampleRateHz = std::max(sampleRateHz_, kMinSampleRateHz);
    const float safeQ = std::max(q_, kMinQ);
    const float nyquistHz = 0.5f * safeSampleRateHz;
    const float safeCenterFreqHz = std::clamp(centerFreqHz_, 0.0f,
                                              nyquistHz - kNyquistMarginHz);
    const float amplitude = std::pow(10.0f, gainDb_ / 40.0f);

    const float w0 = 2.0f * kPi * safeCenterFreqHz / safeSampleRateHz;
    const float cosW0 = std::cos(w0);
    const float sinW0 = std::sin(w0);
    const float alpha = sinW0 / (2.0f * safeQ);

    float b0 = 1.0f + alpha * amplitude;
    float b1 = -2.0f * cosW0;
    float b2 = 1.0f - alpha * amplitude;
    const float a0 = 1.0f + alpha / amplitude;
    float a1 = -2.0f * cosW0;
    float a2 = 1.0f - alpha / amplitude;

    b0 /= a0;
    b1 /= a0;
    b2 /= a0;
    a1 /= a0;
    a2 /= a0;

    coeffs_ << b0, b1, b2, a1, a2;
  }

private:
  FilterCoeff coeffs_ = IIRFilter::identityCoeffs();
  float centerFreqHz_ = 1.0f;
  float q_ = 1.0f;
  float gainDb_ = 0.0f;
  float sampleRateHz_ = 1.0f;
};
