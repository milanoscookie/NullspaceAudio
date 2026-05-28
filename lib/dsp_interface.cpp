#include "dsp_interface.h"

#include "simulator_paths.h"

#include <algorithm>
#include <cmath>
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

void DSPInterface::processWorkerLoop_(std::shared_ptr<ProcessWorkerState> state) {
  while (true) {
    MicBlock micBlock;
    uint64_t seq = 0;
    ProcessMicsFn processor;

    {
      std::unique_lock<std::mutex> lk(state->mailboxMutex);
      state->mailboxCv.wait(lk, [&] { return state->stop || state->jobPending; });

      if (state->stop && !state->jobPending) {
        return;
      }

      micBlock = state->pendingMicBlock;
      seq = state->pendingSeq;
      state->jobPending = false;
      state->workerBusy = true;
    }

    {
      std::lock_guard<std::mutex> lk(state->processorMutex);
      processor = state->processor;
    }

    dsp::Block control = dsp::Block::Zero();
    std::exception_ptr exception;
    try {
      if (processor) {
        processor(micBlock, control);
      }
    } catch (...) {
      exception = std::current_exception();
    }

    {
      std::lock_guard<std::mutex> lk(state->mailboxMutex);
      state->resultReady = true;
      state->resultSeq = seq;
      state->resultControl = control;
      state->resultException = exception;
      state->workerBusy = false;
    }
    state->mailboxCv.notify_all();
  }
}

DSPInterface::DSPInterface(const AudioSourceFactory::Config &audioConfig,
                           int systemLatencyBlocks)
    : DSPInterface(buildDefaultParams(audioConfig), systemLatencyBlocks) {}

DSPInterface::DSPInterface(const Params &params, int systemLatencyBlocks)
    : systemLatencyBlocks_(systemLatencyBlocks), params_(params) {
  H_system_.setImpulseResponse(params_.paths.H);
  P_system_.setImpulseResponse(params_.paths.P);
  C_system_.setImpulseResponse(params_.paths.C);
  speakerSystem_.setImpulseResponse(params_.paths.speaker);
  S_system_.setImpulseResponse(params_.state.S);

  const float centerFreqHz =
      0.5f * (params_.noise.centerFreqMinHz + params_.noise.centerFreqMaxHz);
  const float safeBandwidthHz = std::max(1e-3f, params_.noise.bandwidthHz);
  const float q = std::max(1e-3f, centerFreqHz / safeBandwidthHz);
  noiseBandPassCoeff_ =
      BandPassBiquadCoeff(centerFreqHz, q, static_cast<float>(dsp::SAMPLE_RATE));
  noiseBandPassFilter_.setCoefficients(noiseBandPassCoeff_.getCoefficients());

  const size_t controlBufferSize =
      static_cast<size_t>(std::max(1, systemLatencyBlocks_));
  controlBuf_.resize(controlBufferSize);
  for (size_t i = 0; i < controlBufferSize; ++i) {
    controlBuf_[i] = dsp::Block::Zero();
  }

  inputBuf_.publish(MicBlock{});

  audioSource_ = AudioSourceFactory::create(params_.audioConfig);
  audioSource_->open(
      [this](const dsp::Block &input, dsp::Block &output) {
        audioCallback_(input, output);
      });

  std::thread(&DSPInterface::processWorkerLoop_, processState_).detach();
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

  {
    std::lock_guard<std::mutex> lk(processState_->mailboxMutex);
    processState_->stop = true;
    processState_->jobPending = false;
  }
  processState_->mailboxCv.notify_all();

  {
    std::unique_lock<std::mutex> lk(processState_->mailboxMutex);
    processState_->mailboxCv.wait_for(lk, std::chrono::milliseconds(10), [&] {
      return !processState_->workerBusy && !processState_->jobPending;
    });
  }

  {
    std::lock_guard<std::mutex> lk(processState_->processorMutex);
    processState_->processor = nullptr;
  }

  mic_cv_.notify_all();
}

void DSPInterface::audioCallback_(const dsp::Block &input, dsp::Block &output) {
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
  mb.rawInput = input;
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

  DSPInterface::ObserveMicsFn observeFn;
  {
    std::lock_guard<std::mutex> lk(observeMutex_);
    observeFn = observeMics_;
  }
  if (observeFn) {
    observeFn(mb);
  }

  {
    std::lock_guard<std::mutex> lk(mic_mutex_);
    micQueue_.push_back(mb);
  }
  mic_cv_.notify_all();
}

void DSPInterface::dspThreadLoop_(std::stop_token st) {
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

      const MicBlock *latest = micQueue_.back();
      if (!latest) {
        continue;
      }
      mb = *latest;
      micQueue_.clear();
    }

    const dsp::Block control =
        callProcessMicsWithTimeout_(mb, dsp::BLOCK_LATENCY_US);

    {
      std::lock_guard<std::mutex> lk(controlBufMutex_);
      controlBuf_[controlBufIndex_] = control;
      controlBufIndex_ = (controlBufIndex_ + 1) % controlBuf_.size();
    }
  }
}

void DSPInterface::setProcessMics(ProcessMicsFn fn) {
  std::lock_guard<std::mutex> lk(processState_->processorMutex);
  processState_->processor = std::move(fn);
  startAudioSourceIfNeeded_();
}

void DSPInterface::setObserveMics(DSPInterface::ObserveMicsFn fn) {
  std::lock_guard<std::mutex> lk(observeMutex_);
  observeMics_ = std::move(fn);
}

void DSPInterface::startAudioSourceIfNeeded_() {
  std::lock_guard<std::mutex> lk(startMutex_);
  if (audioSourceStarted_ || !audioSource_) {
    return;
  }
  audioSource_->start();
  audioSourceStarted_ = true;
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
  controlBufIndex_ = (controlBufIndex_ + 1) % controlBuf_.size();
}

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
    updatedS(i) = std::clamp(updatedS(i), -kClip, kClip);
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
  const float sweepPhaseStep =
      1.0f / (sweepHalfPeriodSeconds * static_cast<float>(dsp::SAMPLE_RATE));

  const float trianglePhase = noiseSweepPhase_;
  const float sweepFraction =
      trianglePhase < 1.0f ? trianglePhase : 2.0f - trianglePhase;
  const float sweptCenterFreqHz =
      centerFreqMinHz + centerFreqSpanHz * sweepFraction;

  const float brownianPole =
      std::clamp(params_.noise.centerFreqBrownianPole, 0.0f, 0.99999f);
  const float brownianStddevHz =
      std::max(0.0f, params_.noise.centerFreqBrownianStddevHz);
  const float brownianExcitationStddevHz =
      brownianStddevHz *
      std::sqrt(std::max(0.0f, 1.0f - brownianPole * brownianPole));

  std::normal_distribution<float> brownianDist(0.0f, brownianExcitationStddevHz);
  centerFreqBrownianStateHz_ =
      brownianPole * centerFreqBrownianStateHz_ + brownianDist(noiseRng_);

  const float safeBandwidthHz = std::max(1e-3f, params_.noise.bandwidthHz);
  const float q = std::max(1e-3f, sweptCenterFreqHz / safeBandwidthHz);
  noiseBandPassCoeff_.setCenterFreqHz(sweptCenterFreqHz);
  noiseBandPassCoeff_.setQ(q);
  noiseBandPassFilter_.setCoefficients(noiseBandPassCoeff_.getCoefficients());

  // Deterministic phase-continuous sine sweep. Frequency is advanced per sample
  // so the tone is smooth instead of stair-stepped at block boundaries.
  const float amplitude = sampleSigma * noiseGain;
  for (int i = 0; i < dsp::BLOCK_SIZE; ++i) {
    const float sampleSweepFraction =
        noiseSweepPhase_ < 1.0f ? noiseSweepPhase_ : 2.0f - noiseSweepPhase_;
    const float sampleCenterFreqHz =
        centerFreqMinHz + centerFreqSpanHz * sampleSweepFraction;

    noise(i) = amplitude * std::sin(noiseTonePhase_);
    noiseTonePhase_ += dsp::sim::kTwoPi * sampleCenterFreqHz /
                       static_cast<float>(dsp::SAMPLE_RATE);
    if (noiseTonePhase_ >= dsp::sim::kTwoPi) {
      noiseTonePhase_ -= dsp::sim::kTwoPi;
    }

    noiseSweepPhase_ += sweepPhaseStep;
    if (noiseSweepPhase_ >= 2.0f) {
      noiseSweepPhase_ -= 2.0f;
    }
  }

  noise = noiseBandPassFilter_.filterBlock(noise);

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
  const uint64_t seq = micBlock.seq;

  {
    std::lock_guard<std::mutex> lk(processState_->processorMutex);
    if (!processState_->processor) {
      return result;
    }
  }

  {
    std::lock_guard<std::mutex> lk(processState_->mailboxMutex);
    if (processState_->resultReady && processState_->resultSeq < seq) {
      std::cerr << "ProcessMics late completion for seq "
                << processState_->resultSeq << "; dropping stale output\n";
      processState_->resultReady = false;
      processState_->resultException = nullptr;
    }
    processState_->pendingMicBlock = micBlock;
    processState_->pendingSeq = seq;
    processState_->jobPending = true;
  }
  processState_->mailboxCv.notify_all();

  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::microseconds(timeoutUs);
  std::unique_lock<std::mutex> lk(processState_->mailboxMutex);
  while (true) {
    if (processState_->resultReady) {
      if (processState_->resultSeq == seq) {
        result = processState_->resultControl;
        const std::exception_ptr exception = processState_->resultException;
        processState_->resultReady = false;
        processState_->resultException = nullptr;
        lk.unlock();

        try {
          if (exception) {
            std::rethrow_exception(exception);
          }
        } catch (const std::exception &exceptionObj) {
          std::cerr << "ProcessMics exception: " << exceptionObj.what() << '\n';
          return dsp::Block::Zero();
        } catch (...) {
          std::cerr << "ProcessMics exception: unknown\n";
          return dsp::Block::Zero();
        }

        return result;
      }

      if (processState_->resultSeq < seq) {
        std::cerr << "ProcessMics late completion for seq "
                  << processState_->resultSeq << "; dropping stale output\n";
        processState_->resultReady = false;
        processState_->resultException = nullptr;
        continue;
      }
    }

    if (processState_->mailboxCv.wait_until(lk, deadline) ==
        std::cv_status::timeout) {
      std::cerr << "ProcessMics timeout after " << timeoutUs << " us\n";
      return result;
    }
  }
}
