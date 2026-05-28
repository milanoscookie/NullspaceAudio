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
#include <exception>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <random>
#include <thread>
#include <vector>

using ContextBuffer = RingBuffer<dsp::Block, dsp::CONTEXT_BLOCKS>;

using Clock = std::chrono::steady_clock;

struct alignas(64) MicBlock {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  dsp::Block rawInput = dsp::Block::Zero();
  dsp::Block outside = dsp::Block::Zero();
  dsp::Block inear = dsp::Block::Zero();
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
  using ProcessMicsFn = std::function<void(const MicBlock &, dsp::Block &)>;
  using ObserveMicsFn = std::function<void(const MicBlock &)>;

  DSPInterface(const AudioSourceFactory::Config &audioConfig,
               int systemLatencyBlocks);
  DSPInterface(const Params &params, int systemLatencyBlocks);
  ~DSPInterface();

  DSPInterface(const DSPInterface &) = delete;
  DSPInterface &operator=(const DSPInterface &) = delete;

  int getSystemLatency() const { return systemLatencyBlocks_; }
  void setSystemLatency(int latency) { systemLatencyBlocks_ = latency; }

  std::optional<MicBlock> getMics();
  void sendControl(const dsp::Block &control);

  const Timing &getTiming() const { return params_.timing; }
  const Dynamics &getDynamics() const { return params_.dynamics; }
  const NoiseModel &getNoiseModel() const { return params_.noise; }
  const Paths &getPaths() const { return params_.paths; }

  bool isAudioSourceRunning() const {
    return audioSource_ && audioSource_->isRunning();
  }

  void setProcessMics(ProcessMicsFn fn);
  void setObserveMics(ObserveMicsFn fn);

private:
  struct ProcessWorkerState {
    std::mutex mailboxMutex;
    std::condition_variable mailboxCv;
    bool stop = false;
    bool jobPending = false;
    bool workerBusy = false;
    bool resultReady = false;
    uint64_t pendingSeq = 0;
    uint64_t resultSeq = 0;
    MicBlock pendingMicBlock{};
    dsp::Block resultControl = dsp::Block::Zero();
    std::exception_ptr resultException;

    std::mutex processorMutex;
    ProcessMicsFn processor;
  };

  static void processWorkerLoop_(std::shared_ptr<ProcessWorkerState> state);
  void startAudioSourceIfNeeded_();

  void audioCallback_(const dsp::Block &input, dsp::Block &output);
  void dspThreadLoop_(std::stop_token st);

  void updateDynamicsS_();
  void updateNoiseProfile_();
  void propagatePlant_(const dsp::Block &u, const dsp::Block &referenceNoise,
                       const dsp::Block &inearNoise, MicBlock &mb);
  dsp::Block generateMicNoiseBlock_();
  float computeStddev_(const dsp::Block &block) const;
  dsp::Block callProcessMicsWithTimeout_(const MicBlock &micBlock,
                                         int timeoutUs);

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
  std::mutex startMutex_;
  std::mutex observeMutex_;
  ObserveMicsFn observeMics_;
  std::shared_ptr<ProcessWorkerState> processState_ =
      std::make_shared<ProcessWorkerState>();
  bool audioSourceStarted_ = false;

  std::jthread dspThread_;
  std::unique_ptr<AudioSource> audioSource_;

  Params params_;

  FastLinearSystem<dsp::IR_SIZE> H_system_;
  FastLinearSystem<dsp::IR_SIZE> S_system_;
  FastLinearSystem<dsp::IR_SIZE> C_system_;
  FastLinearSystem<dsp::IR_SIZE> P_system_;
  FastLinearSystem<dsp::IR_SIZE> speakerSystem_;

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
  float noiseTonePhase_ = 0.0f;
  float centerFreqBrownianStateHz_ = 0.0f;

  std::mt19937 noiseRng_{std::random_device{}()};
};
