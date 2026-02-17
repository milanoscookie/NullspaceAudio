#pragma once

#include "dsp_config.h"
#include <Eigen/Dense>

// 2nd Order IIR Filter
class IIRFilter {
public:
  using Block = Eigen::Matrix<float, dsp::BLOCK_SIZE, 1>;
  using FilterCoeff = Eigen::Matrix<float, 5, 1>; // [b0, b1, b2, a1, a2]

  IIRFilter() : coeffs_(identityCoeffs()) {}
  IIRFilter(const FilterCoeff &coeffs) : coeffs_(coeffs) {}

  static FilterCoeff identityCoeffs() {
    FilterCoeff coeffs;
    coeffs << 1.0f, 0.0f, 0.0f, 0.0f, 0.0f;
    return coeffs;
  }

  void setCoefficients(const FilterCoeff &coeffs) { coeffs_ = coeffs; }
  const FilterCoeff &getCoefficients() const { return coeffs_; }

  inline float filterSample(float input) {
    const float output = coeffs_[0] * input + coeffs_[1] * x1_ +
                         coeffs_[2] * x2_ - coeffs_[3] * y1_ - coeffs_[4] * y2_;

    x2_ = x1_;
    x1_ = input;
    y2_ = y1_;
    y1_ = output;

    return output;
  }

  const Block &filterBlock(const Block &input) {
    for (int i = 0; i < dsp::BLOCK_SIZE; ++i) {
      out_(i) = filterSample(input(i));
    }
    return out_;
  };

private:
  FilterCoeff coeffs_ = identityCoeffs();

  float x1_ = 0.0f;
  float x2_ = 0.0f;
  float y1_ = 0.0f;
  float y2_ = 0.0f;

  Block out_ = Block::Zero();
};
