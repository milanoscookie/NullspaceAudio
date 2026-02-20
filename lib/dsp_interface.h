#pragma once

#include "utils/DoubleBufferSPSC.h"
#include "utils/FastLinearSystem.h"
#include "utils/IIRFilter.h"
#include "utils/LPButterworthCoeff.h"
#include "utils/RingBuffer.h"

#include "audio_source.h"
#include "dsp_config.h"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <ctime>
#include <functional>
#include <future>
#include <iostream>
#include <mutex>
#include <optional>
#include <random>
#include <thread>

#include <Eigen/Dense>

using Block = Eigen::Matrix<float, dsp::BLOCK_SIZE, 1>;
using IRBlock = Eigen::Matrix<float, dsp::IR_SIZE, 1>;
using IIRState = Eigen::Matrix<float, dsp::IIR_STATE_SIZE, 1>;

using ContextBuffer = RingBuffer<Block, dsp::CONTEXT_BLOCKS>;

using Clock = std::chrono::steady_clock;

struct MicBlock {
  Block outside;
  Block inear;
  Clock::time_point timestamp = Clock::time_point{};
  uint64_t seq = 0;
};

constexpr size_t MIC_QUEUE_SIZE = 32;

using MicQueue = RingBuffer<MicBlock, MIC_QUEUE_SIZE>;

struct Timing {
  int loop_latency_samp =
      0; // total loop latency (speaker->mics->compute->speaker)
};

struct Dynamics {

  // s = s_true + damping LPF(noise)
  // s renormalized
  // s gets clipped as well to prevent blowing up power
  float noise_gain = 0.001f;
};

struct NoiseModel {
  float outside_mic_stddev = 0.0f;
  float inear_mic_stddev = 0.0f;

  float fc_mean_hz = 500.0f;
  float sigma_fc_hz = 50.0f;
  float fc_lpf_hz = 30.0f;
  float sample_sigma = 0.01f;

  IIRFilter S_noise_filter = IIRFilter(IIRFilter::identityCoeffs());
  IIRFilter noise_color_filter = IIRFilter(IIRFilter::identityCoeffs());
};

struct Paths {
  IRBlock H = IRBlock::Zero();       // noise -> outside mic
  IRBlock P = IRBlock::Zero();       // noise -> in-ear mic
  IRBlock C = IRBlock::Zero();       // speaker -> outside mic
  IRBlock speaker = IRBlock::Zero(); // non-flat speaker response
};

struct State {
  IRBlock S = IRBlock::Zero(); // S_k (evolving transfer function)
  IRBlock S_true = IRBlock::Zero();

  ContextBuffer S_context;

  IIRFilter S_dynamics_ng = IIRFilter(IIRFilter::identityCoeffs());
  IIRFilter mic_noise_color = IIRFilter(IIRFilter::identityCoeffs());
};

struct Params {
  Timing timing;
  Dynamics dynamics;
  NoiseModel noise;
  Paths paths;
  State state;
  AudioSourceFactory::Config audioConfig; // WAV file
};

class DSPInterface {

public:
  DSPInterface(Params &params, int systemLatencyBlocks);

  ~DSPInterface();

  DSPInterface(const DSPInterface &) = delete;
  DSPInterface &operator=(const DSPInterface &) = delete;

  int getSystemLatency() const { return systemLatencyBlocks_; }
  void setSystemLatency(int latency) { systemLatencyBlocks_ = latency; }

  // Read input samples into buffer
  std::optional<MicBlock> getMics();

  // Pass in control noise cancelling signal
  void sendControl(const Block &control);

  const Timing &getTiming() const { return params_.timing; }
  const Dynamics &getDynamics() const { return params_.dynamics; }
  const NoiseModel &getNoiseModel() const { return params_.noise; }
  const Paths &getPaths() const { return params_.paths; }

  // Check if audio source is still running
  bool isAudioSourceRunning() const {
    return audioSource_ && audioSource_->isRunning();
  }

  using ProcessMicsFn = std::function<void(const MicBlock &, Block &)>;
  void setProcessMics(ProcessMicsFn fn);

private:
  int systemLatencyBlocks_ = 1;

  // Ring buffer of control blocks for system latency compensation
  std::vector<Block> controlBuf;
  int controlBufIndex_ = 0;
  std::mutex controlBuf_mutex_;

  // Latest mic block for app observation (getMics)
  DoubleBufferSPSC<MicBlock> inputBuf;

  // Mic blocks queued for DSP thread processing
  std::condition_variable mic_cv_;
  std::mutex mic_mutex_;
  std::atomic<uint64_t> mic_seq_{0};
  MicQueue micQueue_;

  std::mutex paths_mutex_;
  std::mutex process_mutex_;
  ProcessMicsFn processMics_;

  std::jthread dspThread_;

  // Audio source (WAV file)
  std::unique_ptr<AudioSource> audioSource_;
  void audioCallback_(const Block &input, Block &output);

  void step_();            // advance simulation by 1 block
  void updateDynamicsS_(); // update secondary path dynamics (slowly drifting
                           // S_true + noise)
  void
  updateNoiseProfile_(); // update noise model (noise stddev & varying color)

  // Full plant: outside = H*n + C*speaker(u), inear = P*n + S*speaker(u)
  void propagatePlant_(const Block &u, const Block &n, MicBlock &mb);

  Block generateMicNoiseBlock_();
  float computeStddev_(const Block &b) const;

  Params params_;

  FastLinearSystem<dsp::IR_SIZE> H_system_;
  FastLinearSystem<dsp::IR_SIZE> S_system_;
  FastLinearSystem<dsp::IR_SIZE> C_system_;
  FastLinearSystem<dsp::IR_SIZE> P_system_;
  FastLinearSystem<dsp::IR_SIZE> speakerSystem_;

  void dspThreadLoop_(std::stop_token st);
  Block callProcessMicsWithTimeout_(const MicBlock &mb, int timeoutUs);

  // preallocate scratch buffers for plant propagation
  Block u_spk_ = Block::Zero();
  Block yC_ = Block::Zero(); // speaker -> outside mic
  Block yS_ = Block::Zero(); // speaker -> in-ear mic
  Block yH_ = Block::Zero(); // noise -> outside mic
  Block yP_ = Block::Zero(); // noise -> in-ear mic

  Block lastOutside_ = Block::Zero();
  Block lastInear_ = Block::Zero();

  std::mt19937 noiseRng_{std::random_device{}()};
};
