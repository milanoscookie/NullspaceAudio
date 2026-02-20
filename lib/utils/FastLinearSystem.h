#pragma once

#include "RingBuffer.h"
#include "dsp_config.h"
#include <Eigen/Dense>
#include <unsupported/Eigen/FFT>

// Fast convolution using overlap-add FFT method
template <int IR_SIZE> class FastLinearSystem {
public:
  using Block = Eigen::Matrix<float, dsp::BLOCK_SIZE, 1>;
  using IRBlock = Eigen::Matrix<float, IR_SIZE, 1>;

  // FFT size must be >= BLOCK_SIZE + IR_SIZE - 1 for linear convolution
  static constexpr int FFT_SIZE = 2048; // Next power of 2 after 256+1024-1=1279
  static constexpr int OVERLAP_SIZE = IR_SIZE - 1; // 1023 samples
  
  using FFTBlock = Eigen::Matrix<std::complex<float>, FFT_SIZE, 1>;
  using RealFFTBlock = Eigen::Matrix<float, FFT_SIZE, 1>;
  using OverlapBuffer = Eigen::Matrix<float, OVERLAP_SIZE, 1>;

  FastLinearSystem() {
    H_fft_.setZero();
    overlap_.setZero();
  }

  FastLinearSystem(const IRBlock &impulseResponse) : FastLinearSystem() {
    setImpulseResponse(impulseResponse);
  }

  void setImpulseResponse(const IRBlock &impulseResponse) {
    impulseResponse_ = impulseResponse;
    
    // Precompute FFT of impulse response (zero-padded to FFT_SIZE)
    RealFFTBlock h_padded;
    h_padded.setZero();
    h_padded.head(IR_SIZE) = impulseResponse;
    
    fft_.fwd(H_fft_, h_padded);
  }

  const IRBlock &getImpulseResponse() const { return impulseResponse_; }

  // Fast overlap-add convolution
  void step(const Block &input, Block &output) {
    // Zero-pad input to FFT_SIZE
    RealFFTBlock x_padded;
    x_padded.setZero();
    x_padded.head(dsp::BLOCK_SIZE) = input;
    
    // FFT of input
    FFTBlock X_fft;
    fft_.fwd(X_fft, x_padded);
    
    // Frequency-domain multiplication (pointwise)
    FFTBlock Y_fft = X_fft.cwiseProduct(H_fft_);
    
    // IFFT back to time domain
    RealFFTBlock y_full;
    fft_.inv(y_full, Y_fft);
    
    // Overlap-add: first BLOCK_SIZE samples + overlap from previous block
    for (int i = 0; i < dsp::BLOCK_SIZE; ++i) {
      if (i < OVERLAP_SIZE) {
        output(i) = y_full(i) + overlap_(i);
      } else {
        output(i) = y_full(i);
      }
    }
    
    // Save overlap for next block (samples BLOCK_SIZE to BLOCK_SIZE+OVERLAP_SIZE-1)
    overlap_ = y_full.segment<OVERLAP_SIZE>(dsp::BLOCK_SIZE);
  }

private:
  IRBlock impulseResponse_ = IRBlock::Zero();
  FFTBlock H_fft_;
  OverlapBuffer overlap_ = OverlapBuffer::Zero();
  
  Eigen::FFT<float> fft_;
};
