#pragma once

#include "audio_source.h"
#include "dsp_config.h"
#include "utils/BandPassBiquadCoeff.h"
#include "utils/DoubleBufferSPSC.h"
#include "utils/FastLinearSystem.h"
#include "utils/IIRFilter.h"
#include "utils/RingBuffer.h"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <random>
#include <thread>
#include <vector>

using ContextBuffer = RingBuffer<dsp::Block, dsp::CONTEXT_BLOCKS>;

using Clock = std::chrono::steady_clock;

struct MicBlock {
  dsp::Block desiredAudio;
  dsp::Block outside;
  dsp::Block inear;
  Clock::time_point timestamp = Clock::time_point{};
  uint64_t seq = 0;
};

constexpr size_t kMicQueueSize = 32;

using MicQueue = RingBuffer<MicBlock, kMicQueueSize>;

struct Timing {
  int loopLatencySamples = 0;
};

struct Dynamics {
  float noise_gain = dsp::sim::kDefaultModelConfig.secondaryPathDriftGain;
};

struct NoiseModel {
  float outside_mic_stddev = 0.0f;
  float inear_mic_stddev = 0.0f;
  float wav_to_reference_gain = dsp::sim::kDefaultModelConfig.wavToReferenceGain;
  float noise_gain = dsp::sim::kDefaultModelConfig.noise.noiseGain;

  float centerFreqMinHz = dsp::sim::kDefaultModelConfig.noise.centerFreqMinHz;
  float centerFreqMaxHz = dsp::sim::kDefaultModelConfig.noise.centerFreqMaxHz;
  float sweepHalfPeriodSeconds =
      dsp::sim::kDefaultModelConfig.noise.sweepHalfPeriodSeconds;
  float bandwidthHz = dsp::sim::kDefaultModelConfig.noise.bandwidthHz;
  float centerFreqBrownianStddevHz =
      dsp::sim::kDefaultModelConfig.noise.centerFreqBrownianStddevHz;
  float centerFreqBrownianPole =
      dsp::sim::kDefaultModelConfig.noise.centerFreqBrownianPole;
  float sample_sigma = dsp::sim::kDefaultModelConfig.noise.sampleSigma;
};

struct Paths {
  dsp::IRBlock H = dsp::IRBlock::Zero();
  dsp::IRBlock P = dsp::IRBlock::Zero();
  dsp::IRBlock C = dsp::IRBlock::Zero();
  dsp::IRBlock speaker = dsp::IRBlock::Zero();
};

struct State {
  dsp::IRBlock S = dsp::IRBlock::Zero();
  dsp::IRBlock S_true = dsp::IRBlock::Zero();

  ContextBuffer S_context;

  IIRFilter S_dynamics_ng = IIRFilter(IIRFilter::identityCoeffs());
  IIRFilter mic_noise_color = IIRFilter(IIRFilter::identityCoeffs());
};

struct Params {
  Params();
  explicit Params(const AudioSourceFactory::Config &audioConfig);

  Timing timing;
  Dynamics dynamics;
  NoiseModel noise;
  Paths paths;
  State state;
  AudioSourceFactory::Config audioConfig;
};

class DSPInterface {
public:
  DSPInterface(const AudioSourceFactory::Config &audioConfig,
               int systemLatencyBlocks);
  DSPInterface(const Params &params, int systemLatencyBlocks);
  ~DSPInterface();

  DSPInterface(const DSPInterface &) = delete;
  DSPInterface &operator=(const DSPInterface &) = delete;

  // RT-safe.
  // Returns the control-to-output latency in blocks.
  int getSystemLatency() const { return systemLatencyBlocks_; }

  // RT-unsafe.
  // Updates the control-to-output latency and resizes related buffering.
  void setSystemLatency(int latency) { systemLatencyBlocks_ = latency; }

  // RT-safe.
  // Returns the latest published microphone block if one is available.
  std::optional<MicBlock> getMics();

  // RT-safe.
  // Publishes one control block for plant propagation.
  void sendControl(const dsp::Block &control);

  // RT-safe.
  const Timing &getTiming() const { return params_.timing; }

  // RT-safe.
  const Dynamics &getDynamics() const { return params_.dynamics; }

  // RT-safe.
  const NoiseModel &getNoiseModel() const { return params_.noise; }

  // RT-safe.
  const Paths &getPaths() const { return params_.paths; }

  // RT-safe.
  // Returns whether the backing audio source is still running.
  bool isAudioSourceRunning() const {
    return audioSource_ && audioSource_->isRunning();
  }

  using ProcessMicsFn = std::function<void(const MicBlock &, dsp::Block &)>;

  // RT-unsafe.
  // Registers the microphone processing function used by the worker thread.
  void setProcessMics(ProcessMicsFn fn);

private:
  int systemLatencyBlocks_ = 1;

  std::vector<dsp::Block> controlBuf_;
  int controlBufIndex_ = 0;
  std::mutex controlBufMutex_;

  DoubleBufferSPSC<MicBlock> inputBuf_;

  std::condition_variable mic_cv_;
  std::mutex mic_mutex_;
  std::atomic<uint64_t> mic_seq_{0};
  MicQueue micQueue_;

  std::mutex pathsMutex_;
  std::mutex processMutex_;
  ProcessMicsFn processMics_;
  std::shared_ptr<std::atomic<bool>> processMicsBusy_ =
      std::make_shared<std::atomic<bool>>(false);

  std::jthread dspThread_;

  std::unique_ptr<AudioSource> audioSource_;
  void audioCallback_(const dsp::Block &input, dsp::Block &output);

  void step_();
  void updateDynamicsS_();
  void updateNoiseProfile_();

  void propagatePlant_(const dsp::Block &u,
                       const dsp::Block &referenceNoise,
                       const dsp::Block &inearNoise, MicBlock &mb);

  dsp::Block generateMicNoiseBlock_();
  float computeStddev_(const dsp::Block &block) const;

  Params params_;

  FastLinearSystem<dsp::IR_SIZE> H_system_;
  FastLinearSystem<dsp::IR_SIZE> S_system_;
  FastLinearSystem<dsp::IR_SIZE> C_system_;
  FastLinearSystem<dsp::IR_SIZE> P_system_;
  FastLinearSystem<dsp::IR_SIZE> speakerSystem_;

  void dspThreadLoop_(std::stop_token st);
  dsp::Block callProcessMicsWithTimeout_(const MicBlock &micBlock,
                                         int timeoutUs);

  dsp::Block u_spk_ = dsp::Block::Zero();
  dsp::Block yC_ = dsp::Block::Zero();
  dsp::Block yS_ = dsp::Block::Zero();
  dsp::Block yH_ = dsp::Block::Zero();
  dsp::Block yP_ = dsp::Block::Zero();

  dsp::Block lastOutside_ = dsp::Block::Zero();
  dsp::Block lastInear_ = dsp::Block::Zero();

  IIRFilter noiseBandPassFilter_ = IIRFilter(IIRFilter::identityCoeffs());
  BandPassBiquadCoeff noiseBandPassCoeff_;
  float noiseSweepPhase_ = 0.0f;
  float centerFreqBrownianStateHz_ = 0.0f;

  std::mt19937 noiseRng_{std::random_device{}()};
};
