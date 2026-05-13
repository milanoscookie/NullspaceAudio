#include "dsp_interface.h"

#include "simulator_paths.h"

#include <algorithm>
#include <cmath>
#include <future>
#include <iostream>

namespace {

Params buildDefaultParams(const AudioSourceFactory::Config &audioConfig) {
  Params params;
  params.audioConfig = audioConfig;
  return params;
}

} // namespace

Params::Params() {
  const auto pathSet = dsp::sim::buildPathSet();
  paths.H = pathSet.reference;
  paths.P = pathSet.primary;
  paths.C = pathSet.feedback;
  paths.speaker = pathSet.speaker;
  state.S = pathSet.secondary;
  state.S_true = pathSet.secondary;
  noise.wav_to_reference_gain = dsp::sim::kDefaultModelConfig.wavToReferenceGain;
}

Params::Params(const AudioSourceFactory::Config &audioConfig) : Params() {
  this->audioConfig = audioConfig;
}

DSPInterface::DSPInterface(const AudioSourceFactory::Config &audioConfig,
                           int systemLatencyBlocks)
    : DSPInterface(buildDefaultParams(audioConfig), systemLatencyBlocks) {}

DSPInterface::DSPInterface(const Params &params, int systemLatencyBlocks)
    : params_(params), systemLatencyBlocks_(systemLatencyBlocks) {

  H_system_.setImpulseResponse(params.paths.H);
  P_system_.setImpulseResponse(params.paths.P);
  C_system_.setImpulseResponse(params.paths.C);
  speakerSystem_.setImpulseResponse(params.paths.speaker);
  S_system_.setImpulseResponse(params.state.S);

  const float centerFreqHz =
      0.5f * (params_.noise.centerFreqMinHz + params_.noise.centerFreqMaxHz);
  const float safeBandwidthHz = std::max(1e-3f, params_.noise.bandwidthHz);
  const float q = std::max(1e-3f, centerFreqHz / safeBandwidthHz);
  noiseBandPassCoeff_ =
      BandPassBiquadCoeff(centerFreqHz, q, static_cast<float>(dsp::SAMPLE_RATE));
  noiseBandPassFilter_.setCoefficients(noiseBandPassCoeff_.getCoefficients());

  controlBuf_.resize(systemLatencyBlocks_);
  for (int i = 0; i < systemLatencyBlocks_; ++i) {
    controlBuf_[i] = dsp::Block::Zero();
  }
  controlBufIndex_ = 0;

  inputBuf_.publish(MicBlock{.desiredAudio = dsp::Block::Zero(),
                             .outside = dsp::Block::Zero(),
                             .inear = dsp::Block::Zero()});

  audioSource_ = AudioSourceFactory::create(params.audioConfig);

  audioSource_->open([this](const dsp::Block &input, dsp::Block &output) {
    audioCallback_(input, output);
  });
  audioSource_->start();

  dspThread_ = std::jthread([this](std::stop_token st) { dspThreadLoop_(st); });
}

DSPInterface::~DSPInterface() {
  if (audioSource_) {
    audioSource_->stop();
    audioSource_->close();
  }

  if (dspThread_.joinable()) {
    dspThread_.request_stop();
  }

  mic_cv_.notify_all();
}

void DSPInterface::audioCallback_(const dsp::Block &input,
                                  dsp::Block &output) {
  dsp::Block u = dsp::Block::Zero();
  {
    std::lock_guard<std::mutex> lk(controlBufMutex_);
    u = controlBuf_[controlBufIndex_];
  }

  const dsp::Block syntheticNoise = generateMicNoiseBlock_();
  const dsp::Block referenceExcitation =
      params_.noise.wav_to_reference_gain * input + syntheticNoise;
  const dsp::Block inearExcitation = input + syntheticNoise;

  updateDynamicsS_();

  MicBlock mb;
  mb.desiredAudio = input;
  mb.timestamp = Clock::now();
  mb.seq = mic_seq_.fetch_add(1, std::memory_order_relaxed) + 1;
  {
    std::lock_guard<std::mutex> lk(pathsMutex_);
    propagatePlant_(u, referenceExcitation, inearExcitation, mb);
  }

  output = mb.inear;

  lastOutside_ = mb.outside;
  lastInear_ = mb.inear;
  updateNoiseProfile_();

  inputBuf_.publish(mb);

  {
    std::lock_guard<std::mutex> lk(mic_mutex_);
    micQueue_.push_back(mb);
  }
  mic_cv_.notify_all();
}

void DSPInterface::dspThreadLoop_(std::stop_token st) {
  dsp::Block control = dsp::Block::Zero();
  while (!st.stop_requested()) {
    MicBlock mb;
    {
      std::unique_lock<std::mutex> lk(mic_mutex_);
      mic_cv_.wait(lk, [&] {
        return st.stop_requested() || micQueue_.front() != nullptr;
      });

      if (st.stop_requested()) {
        return;
      }

      const MicBlock *p = micQueue_.front();
      if (!p) {
        continue;
      }
      mb = *p;
      micQueue_.pop_front();
    }

    control = callProcessMicsWithTimeout_(mb, dsp::BLOCK_LATENCY_US);

    {
      std::lock_guard<std::mutex> lk(controlBufMutex_);
      controlBuf_[controlBufIndex_] = control;
      controlBufIndex_ = (controlBufIndex_ + 1) % systemLatencyBlocks_;
    }
  }
}

void DSPInterface::setProcessMics(ProcessMicsFn fn) {
  std::lock_guard<std::mutex> lk(processMutex_);
  processMics_ = std::move(fn);
}

std::optional<MicBlock> DSPInterface::getMics() {
  MicBlock mb;
  if (inputBuf_.tryRead(mb)) {
    return mb;
  }
  return std::nullopt;
}

void DSPInterface::sendControl(const dsp::Block &control) {
  std::lock_guard<std::mutex> lk(controlBufMutex_);
  controlBuf_[controlBufIndex_] = control;
  controlBufIndex_ = (controlBufIndex_ + 1) % systemLatencyBlocks_;
}

void DSPInterface::step_() {}

void DSPInterface::updateNoiseProfile_() {
  auto &noise = params_.noise;
  noise.outside_mic_stddev = computeStddev_(lastOutside_);
  noise.inear_mic_stddev = computeStddev_(lastInear_);
}

void DSPInterface::updateDynamicsS_() {
  auto &state = params_.state;
  auto &dyn = params_.dynamics;

  static bool initialized = false;
  static dsp::IRBlock sTrue = state.S_true;

  if (!initialized) {
    sTrue = state.S;
    initialized = true;
  }

  dsp::IRBlock filteredNoise;
  constexpr int kNumBlocks = dsp::IR_SIZE / dsp::BLOCK_SIZE;
  for (int i = 0; i < kNumBlocks; ++i) {
    dsp::Block noiseBlock = dsp::Block::Random();
    const dsp::Block &filteredBlock =
        state.S_dynamics_ng.filterBlock(noiseBlock);
    filteredNoise.segment<dsp::BLOCK_SIZE>(i * dsp::BLOCK_SIZE) = filteredBlock;
  }

  dsp::IRBlock updatedS = sTrue + dyn.noise_gain * filteredNoise;

  constexpr float kEpsilon = 1e-8f;
  const float originalNorm = std::sqrt(sTrue.squaredNorm());
  const float updatedNorm = std::sqrt(updatedS.squaredNorm());
  if (originalNorm > kEpsilon && updatedNorm > kEpsilon) {
    updatedS *= (originalNorm / updatedNorm);
  }

  constexpr float kClip = 1.0f;
  for (int i = 0; i < updatedS.size(); ++i) {
    if (updatedS(i) > kClip) {
      updatedS(i) = kClip;
    }
    if (updatedS(i) < -kClip) {
      updatedS(i) = -kClip;
    }
  }

  state.S = updatedS;
  {
    std::lock_guard<std::mutex> lk(pathsMutex_);
    S_system_.setImpulseResponse(state.S);
  }
}

void DSPInterface::propagatePlant_(const dsp::Block &u,
                                   const dsp::Block &referenceNoise,
                                   const dsp::Block &inearNoise,
                                   MicBlock &mb) {
  speakerSystem_.step(u, u_spk_);
  C_system_.step(u_spk_, yC_);
  S_system_.step(u_spk_, yS_);
  H_system_.step(referenceNoise, yH_);
  P_system_.step(inearNoise, yP_);
  mb.outside = yH_;
  mb.outside += yC_;
  mb.inear = yP_;
  mb.inear += yS_;
}

dsp::Block DSPInterface::generateMicNoiseBlock_() {
  dsp::Block noise = dsp::Block::Zero();
  const float sampleSigma = params_.noise.sample_sigma;
  const float noiseGain = params_.noise.noise_gain;
  if (sampleSigma <= 0.0f || noiseGain <= 0.0f) {
    return noise;
  }

  const float centerFreqMinHz = params_.noise.centerFreqMinHz;
  const float centerFreqMaxHz = params_.noise.centerFreqMaxHz;
  const float centerFreqSpanHz = std::max(0.0f, centerFreqMaxHz - centerFreqMinHz);
  const float sweepHalfPeriodSeconds =
      std::max(1e-6f, params_.noise.sweepHalfPeriodSeconds);
  const float blockDurationSeconds =
      static_cast<float>(dsp::BLOCK_SIZE) / static_cast<float>(dsp::SAMPLE_RATE);
  const float phaseStep = blockDurationSeconds / sweepHalfPeriodSeconds;

  float trianglePhase = noiseSweepPhase_;
  float sweepFraction = 0.0f;
  if (trianglePhase < 1.0f) {
    sweepFraction = trianglePhase;
  } else {
    sweepFraction = 2.0f - trianglePhase;
  }
  const float sweptCenterFreqHz =
      centerFreqMinHz + centerFreqSpanHz * sweepFraction;

  const float brownianPole =
      std::clamp(params_.noise.centerFreqBrownianPole, 0.0f, 0.99999f);
  const float brownianStddevHz =
      std::max(0.0f, params_.noise.centerFreqBrownianStddevHz);
  const float brownianExcitationStddevHz =
      brownianStddevHz * std::sqrt(std::max(0.0f, 1.0f - brownianPole * brownianPole));

  std::normal_distribution<float> brownianDist(0.0f, brownianExcitationStddevHz);
  centerFreqBrownianStateHz_ =
      brownianPole * centerFreqBrownianStateHz_ + brownianDist(noiseRng_);

  const float noisyCenterFreqHz = std::clamp(
      sweptCenterFreqHz + centerFreqBrownianStateHz_, centerFreqMinHz,
      centerFreqMaxHz);
  const float safeBandwidthHz = std::max(1e-3f, params_.noise.bandwidthHz);
  const float q = std::max(1e-3f, noisyCenterFreqHz / safeBandwidthHz);
  noiseBandPassCoeff_.setCenterFreqHz(noisyCenterFreqHz);
  noiseBandPassCoeff_.setQ(q);
  noiseBandPassFilter_.setCoefficients(noiseBandPassCoeff_.getCoefficients());

  std::normal_distribution<float> noiseDist(0.0f, sampleSigma);
  for (int i = 0; i < dsp::BLOCK_SIZE; ++i) {
    noise(i) = noiseDist(noiseRng_);
  }

  noise = noiseBandPassFilter_.filterBlock(noise);

  const float mean = noise.mean();
  noise.array() -= mean;
  const float peak = noise.cwiseAbs().maxCoeff();
  if (peak > 1e-12f) {
    noise *= (sampleSigma / peak);
  }
  noise *= noiseGain;

  noiseSweepPhase_ += phaseStep;
  if (noiseSweepPhase_ >= 2.0f) {
    noiseSweepPhase_ -= 2.0f;
  }

  return noise;
}

float DSPInterface::computeStddev_(const dsp::Block &block) const {
  const float mean = block.mean();
  const float var = (block.array() - mean).square().mean();
  return std::sqrt(var);
}

dsp::Block DSPInterface::callProcessMicsWithTimeout_(const MicBlock &micBlock,
                                                     int timeoutUs) {
  dsp::Block result = dsp::Block::Zero();

  ProcessMicsFn fn;
  {
    std::lock_guard<std::mutex> lk(processMutex_);
    fn = processMics_;
  }

  if (!fn) {
    return result;
  }

  auto busy = processMicsBusy_;
  bool expected = false;
  if (!busy->compare_exchange_strong(expected, true,
                                     std::memory_order_acq_rel)) {
    return result;
  }

  std::promise<dsp::Block> promise;
  auto task = promise.get_future();
  std::thread(
      [fn = std::move(fn), micBlock, promise = std::move(promise), busy]() mutable {
        dsp::Block control = dsp::Block::Zero();
        try {
          fn(micBlock, control);
          promise.set_value(control);
        } catch (...) {
          try {
            promise.set_exception(std::current_exception());
          } catch (...) {
          }
        }
        busy->store(false, std::memory_order_release);
      })
      .detach();

  const auto deadline = std::chrono::microseconds(timeoutUs);
  if (task.wait_for(deadline) == std::future_status::timeout) {
    std::cerr << "ProcessMics timeout after " << timeoutUs << " us\n";
    return result;
  }

  try {
    result = task.get();
  } catch (const std::exception &exception) {
    std::cerr << "ProcessMics exception: " << exception.what() << '\n';
  } catch (...) {
    std::cerr << "ProcessMics exception: unknown\n";
  }

  return result;
}
