#pragma once

#include "dsp_config.h"

// Fast convolution using overlap-add FFT method
template <int IR_SIZE> class FastLinearSystem {
public:
  using Block = dsp::Block;
  using IRBlock = Eigen::Matrix<float, IR_SIZE, 1>;

  static constexpr int convolutionSize() {
    int size = 1;
    const int requiredSize = dsp::BLOCK_SIZE + IR_SIZE - 1;
    while (size < requiredSize) {
      size <<= 1;
    }
    return size;
  }

  static constexpr int FFT_SIZE = convolutionSize();
  static constexpr int OVERLAP_SIZE = IR_SIZE - 1;

  using FFTBlock = Eigen::Matrix<std::complex<float>, FFT_SIZE, 1>;
  using RealFFTBlock = Eigen::Matrix<float, FFT_SIZE, 1>;
  using OverlapBuffer = Eigen::Matrix<float, OVERLAP_SIZE, 1>;

  static_assert(FFT_SIZE >= dsp::BLOCK_SIZE + OVERLAP_SIZE,
                "FFT_SIZE must hold a full block convolution result");

  FastLinearSystem() {
    H_fft_.setZero();
    overlap_.setZero();
    x_padded_.setZero();
  }

  FastLinearSystem(const IRBlock &impulseResponse) : FastLinearSystem() {
    setImpulseResponse(impulseResponse);
  }

  void setImpulseResponse(const IRBlock &impulseResponse) {
    impulseResponse_ = impulseResponse;

    RealFFTBlock h_padded;
    h_padded.setZero();
    h_padded.head(IR_SIZE) = impulseResponse;

    fft_.fwd(H_fft_, h_padded);
  }

  const IRBlock &getImpulseResponse() const { return impulseResponse_; }

  void step(const Block &input, Block &output) {
    x_padded_.head(dsp::BLOCK_SIZE) = input;

    fft_.fwd(X_fft_, x_padded_);

    Y_fft_ = X_fft_.cwiseProduct(H_fft_);

    fft_.inv(y_full_, Y_fft_);

    for (int i = 0; i < dsp::BLOCK_SIZE; ++i) {
      if (i < OVERLAP_SIZE) {
        output(i) = y_full_(i) + overlap_(i);
      } else {
        output(i) = y_full_(i);
      }
    }

    overlap_ = y_full_.template segment<OVERLAP_SIZE>(dsp::BLOCK_SIZE);
  }

private:
  IRBlock impulseResponse_ = IRBlock::Zero();
  FFTBlock H_fft_;
  OverlapBuffer overlap_ = OverlapBuffer::Zero();

  Eigen::FFT<float> fft_;
  RealFFTBlock x_padded_;
  FFTBlock X_fft_;
  FFTBlock Y_fft_;
  RealFFTBlock y_full_;
};
