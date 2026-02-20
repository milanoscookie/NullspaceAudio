#include "dsp_interface.h"

DSPInterface::DSPInterface(Params &params, int systemLatencyBlocks)
    : params_(params), systemLatencyBlocks_(systemLatencyBlocks) {

  H_system_.setImpulseResponse(params.paths.H);
  P_system_.setImpulseResponse(params.paths.P);
  C_system_.setImpulseResponse(params.paths.C);
  speakerSystem_.setImpulseResponse(params.paths.speaker);
  S_system_.setImpulseResponse(params.state.S);

  LPButterworthCoeff noiseFcLpf(params_.noise.fc_lpf_hz,
                                static_cast<float>(dsp::SAMPLE_RATE));
  params_.noise.noise_color_filter.setCoefficients(
      noiseFcLpf.getCoefficients());

  // Initialize control ring buffer with systemLatencyBlocks_ elements
  controlBuf.resize(systemLatencyBlocks_);
  for (int i = 0; i < systemLatencyBlocks_; ++i) {
    controlBuf[i] = Block::Zero();
  }
  controlBufIndex_ = 0;
  
  inputBuf.publish(MicBlock{Block::Zero(), Block::Zero()});

  // Create audio source
  audioSource_ = AudioSourceFactory::create(params.audioConfig);

  audioSource_->open([this](const Block &input, Block &output) {
    audioCallback_(input, output);
  });
  audioSource_->start();

  // ANC processing thread
  dspThread_ = std::jthread([this](std::stop_token st) { dspThreadLoop_(st); });
}
DSPInterface::~DSPInterface() {
  // Stop audio source first
  if (audioSource_) {
    audioSource_->stop();
    audioSource_->close();
  }

  if (dspThread_.joinable())
    dspThread_.request_stop();

  mic_cv_.notify_all();
}

void DSPInterface::audioCallback_(const Block &input, Block &output) {
  // read command signal U from the ring buffer (delayed by systemLatencyBlocks_)
  Block u = Block::Zero();
  {
    std::lock_guard<std::mutex> lk(controlBuf_mutex_);
    u = controlBuf[controlBufIndex_];
  }

  // simulate ambientNoise
  const Block ambientNoise = input + generateMicNoiseBlock_();

  // update S using a slowly drifting secondary path
  updateDynamicsS_();

  // Propagate full plant with previous u and current noise
  //    outside = H*n + C*speaker(u)
  //    inear   = P*n + S*speaker(u)
  MicBlock mb;
  mb.timestamp = Clock::now();
  mb.seq = mic_seq_.fetch_add(1, std::memory_order_relaxed) + 1;
  {
    std::lock_guard<std::mutex> lk(paths_mutex_);
    propagatePlant_(u, ambientNoise, mb);
  }

  // the output is the inear mic. This is what the user hears and what we cares
  // about
  output = mb.inear;

  // Update noise profile statistics
  lastOutside_ = mb.outside;
  lastInear_ = mb.inear;
  updateNoiseProfile_();

  // publishMics to the inputBuf
  inputBuf.publish(mb);

  // enque dspthread for the next processing step
  {
    std::lock_guard<std::mutex> lk(mic_mutex_);
    micQueue_.push_back(mb);
  }
  mic_cv_.notify_all();
}

void DSPInterface::dspThreadLoop_(std::stop_token st) {
  Block control = Block::Zero();
  while (!st.stop_requested()) {
    MicBlock mb;
    {
      std::unique_lock<std::mutex> lk(mic_mutex_);
      mic_cv_.wait(lk, [&] {
        return st.stop_requested() || micQueue_.front() != nullptr;
      });

      if (st.stop_requested())
        return;

      const MicBlock *p = micQueue_.front();
      if (!p)
        continue;
      mb = *p;
      micQueue_.pop_front();
    }

    control = callProcessMicsWithTimeout_(mb, dsp::BLOCK_LATENCY_US);

    // Publish speaker command to ring buffer
    {
      std::lock_guard<std::mutex> lk(controlBuf_mutex_);
      controlBuf[controlBufIndex_] = control;
      controlBufIndex_ = (controlBufIndex_ + 1) % systemLatencyBlocks_;
    }
  }
}

void DSPInterface::setProcessMics(ProcessMicsFn fn) {
  std::lock_guard<std::mutex> lk(process_mutex_);
  processMics_ = std::move(fn);
}

std::optional<MicBlock> DSPInterface::getMics() {
  MicBlock mb;
  if (inputBuf.tryRead(mb)) {
    return mb;
  }
  return std::nullopt;
}
void DSPInterface::sendControl(const Block &control) {
  std::lock_guard<std::mutex> lk(controlBuf_mutex_);
  controlBuf[controlBufIndex_] = control;
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
  static IRBlock S_true = state.S_true;

  if (!initialized) {
    S_true = state.S;
    initialized = true;
  }

  // Generate filtered white noise for the entire IR
  IRBlock w_lp;
  constexpr int num_blocks = dsp::IR_SIZE / dsp::BLOCK_SIZE;
  for (int i = 0; i < num_blocks; ++i) {
    Block w_block = Block::Random();
    Block w_lp_block = state.S_dynamics_ng.filterBlock(w_block);
    w_lp.segment<dsp::BLOCK_SIZE>(i * dsp::BLOCK_SIZE) = w_lp_block;
  }
  
  IRBlock S_new = S_true + dyn.noise_gain * w_lp;

  // renormalize
  const float eps = 1e-12f;
  const float n0 = std::sqrt(S_true.squaredNorm());
  const float n1 = std::sqrt(S_new.squaredNorm());
  if (n0 > eps && n1 > eps) {
    S_new *= (n0 / n1);
  }

  // clip
  constexpr float clip = 1.0f;
  for (int i = 0; i < S_new.size(); ++i) {
    if (S_new(i) > clip)
      S_new(i) = clip;
    if (S_new(i) < -clip)
      S_new(i) = -clip;
  }

  state.S = S_new;
  {
    std::lock_guard<std::mutex> lk(paths_mutex_);
    S_system_.setImpulseResponse(state.S);
  }
}

void DSPInterface::propagatePlant_(const Block &u, const Block &n,
                                   MicBlock &mb) {
  // Speaker coloration
  speakerSystem_.step(u, u_spk_);

  // Speaker -> outside mic (C path) and speaker -> in-ear mic (S path)
  C_system_.step(u_spk_, yC_);
  S_system_.step(u_spk_, yS_);

  // Noise -> outside mic (H path) and noise -> in-ear mic (P path)
  H_system_.step(n, yH_);
  P_system_.step(n, yP_);

  // outside_mic = H*n + C*speaker(u)
  mb.outside = yH_;
  mb.outside += yC_;

  // inear_mic = P*n + S*speaker(u)
  mb.inear = yP_;
  mb.inear += yS_;
}

Block DSPInterface::generateMicNoiseBlock_() {
  Block fcRandom = Block::Zero();
  const float fcMean = params_.noise.fc_mean_hz;
  const float sigmaFc = std::max(1e-6f, params_.noise.sigma_fc_hz);

  std::normal_distribution<float> fcDist(fcMean, sigmaFc);
  for (int i = 0; i < dsp::BLOCK_SIZE; ++i) {
    fcRandom(i) = std::abs(fcDist(noiseRng_));
  }

  Block fcLowPassed = params_.noise.noise_color_filter.filterBlock(fcRandom);
  Block fcForNoise = fcLowPassed.cwiseAbs();

  Block noise = Block::Zero();
  const float sampleSigma = std::max(1e-6f, params_.noise.sample_sigma);
  for (int i = 0; i < dsp::BLOCK_SIZE; ++i) {
    std::normal_distribution<float> nDist(fcForNoise(i), sampleSigma);
    noise(i) = nDist(noiseRng_);
  }

  // Normalize to audio range: zero-mean, then scale so peak ≈ sample_sigma
  const float mean = noise.mean();
  noise.array() -= mean;
  const float peak = noise.cwiseAbs().maxCoeff();
  if (peak > 1e-12f) {
    noise *= (sampleSigma / peak);
  }

  return noise;
}

float DSPInterface::computeStddev_(const Block &b) const {
  const float mean = b.mean();
  const float var = (b.array() - mean).square().mean();
  return std::sqrt(var);
}

Block DSPInterface::callProcessMicsWithTimeout_(const MicBlock &mb,
                                                int timeoutUs) {
  Block result = Block::Zero();

  ProcessMicsFn fn;
  {
    std::lock_guard<std::mutex> lk(process_mutex_);
    fn = processMics_;
  }

  if (!fn)
    return result;

  auto task = std::async(std::launch::async, [&]() { fn(mb, result); });

  auto deadline = std::chrono::microseconds(timeoutUs);
  if (task.wait_for(deadline) == std::future_status::timeout) {
    std::cerr << "ProcessMics timeout after " << timeoutUs << " us\n";
    result = Block::Zero();
  }

  return result;
}
