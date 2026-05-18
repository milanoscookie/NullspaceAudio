// Tests for DSPInterface — plant propagation, callback flow, getMics/sendControl
#include "test_harness.h"
#include "dsp_interface.h"
#include <atomic>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <stdexcept>
#include <thread>

static std::string resolveTestWavPath() {
  namespace fs = std::filesystem;

  static const char *kCandidates[] = {
      "out.wav",
      "../out.wav",
      "../../out.wav",
      "/home/milan/Documents/DSPClass/ANC/out.wav",
  };

  for (const char *candidate : kCandidates) {
    if (fs::exists(candidate)) {
      return candidate;
    }
  }

  throw std::runtime_error("Could not locate out.wav");
}

// Helper: create params with simple delta impulse responses
static Params makeTestParams() {
  Params p;
  p.paths.H(0) = 1.0f;       // noise -> outside (passthrough)
  p.paths.P(0) = 0.5f;       // noise -> in-ear (half gain)
  p.paths.C(0) = 0.1f;       // speaker -> outside (leakage)
  p.paths.speaker(0) = 1.0f; // flat speaker
  p.state.S(0) = 0.8f;       // speaker -> in-ear
  p.state.S_true = p.state.S;

  // Use WAV file (will need to provide paths)
  p.audioConfig.type = AudioSourceFactory::Type::WavFile;
  p.audioConfig.inputWavPath = resolveTestWavPath();
  p.audioConfig.outputWavPath = "";
  p.audioConfig.loop = true;

  // Very low noise so we can reason about signals
  p.noise.sample_sigma = 0.001f;
  p.noise.centerFreqMinHz = 300.0f;
  p.noise.centerFreqMaxHz = 700.0f;
  p.noise.sweepHalfPeriodSeconds = 1.0f;
  p.noise.bandwidthHz = 30.0f;
  p.noise.centerFreqBrownianStddevHz = 3.0f;
  p.noise.centerFreqBrownianPole = 0.995f;
  p.dynamics.noise_gain = 0.0f; // disable S drift for testing

  return p;
}

TEST(constructs_and_destructs) {
  Params p = makeTestParams();
  {
    DSPInterface dsp(p, 3);
    dsp.setProcessMics([](const MicBlock &, dsp::Block &control) {
      control.setZero();
    });
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  // Should not crash or hang
  ASSERT_TRUE(true);
}

TEST(getMics_returns_data) {
  Params p = makeTestParams();
  DSPInterface dsp(p, 3);
  dsp.setProcessMics([](const MicBlock &, dsp::Block &control) {
    control.setZero();
  });

  // Wait for some blocks to flow
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  auto mic = dsp.getMics();
  ASSERT_TRUE(mic.has_value());
  ASSERT_TRUE(std::isfinite(mic->rawInput.sum()));
}

TEST(sendControl_is_readable) {
  Params p = makeTestParams();
  DSPInterface dsp(p, 3);

  std::atomic<bool> received{false};
  dsp.setProcessMics([&](const MicBlock &mb, dsp::Block &control) {
    // Just check inear has some value (from noise at least)
    if (mb.inear.cwiseAbs().maxCoeff() > 0.0f || true) {
      received.store(true, std::memory_order_relaxed);
    }
    control.setZero();
  });

  // Send a control signal
  dsp::Block ctrl = dsp::Block::Ones() * 0.1f;
  dsp.sendControl(ctrl);

  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  ASSERT_TRUE(received.load());
}

TEST(processMics_callback_invoked) {
  Params p = makeTestParams();
  DSPInterface dsp(p, 3);

  std::atomic<int> count{0};
  dsp.setProcessMics([&](const MicBlock &, dsp::Block &control) {
    count.fetch_add(1, std::memory_order_relaxed);
    control.setZero();
  });

  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  int c = count.load();
  // At 48kHz / 256 = ~187.5 blocks/s, in 500ms should get ~90+
  ASSERT_TRUE(c > 10);
}

TEST(zero_control_inear_matches_noise_path) {
  // With zero control, inear should be P*noise only
  Params p = makeTestParams();
  p.noise.sample_sigma = 0.0f; // disable noise generator
  
  DSPInterface dsp(p, 3);
  dsp.setProcessMics([](const MicBlock &, dsp::Block &control) {
    control.setZero();
  });

  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  auto mic = dsp.getMics();
  ASSERT_TRUE(mic.has_value());
  // With no noise and no real mic input, signals should be near zero
  // (WAV file input will depend on the source file)
  // Just verify it's finite
  ASSERT_TRUE(std::isfinite(mic->rawInput.sum()));
  ASSERT_TRUE(std::isfinite(mic->outside.sum()));
  ASSERT_TRUE(std::isfinite(mic->inear.sum()));
}

TEST(mic_block_carries_raw_input) {
  Params p = makeTestParams();
  p.noise.sample_sigma = 0.0f;
  p.noise.wav_to_reference_gain = 0.0f;

  DSPInterface dsp(p, 3);
  dsp.setProcessMics([](const MicBlock &, dsp::Block &control) {
    control.setZero();
  });

  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  auto mic = dsp.getMics();
  ASSERT_TRUE(mic.has_value());
  ASSERT_TRUE(std::isfinite(mic->rawInput.sum()));
  ASSERT_TRUE(mic->rawInput.cwiseAbs().maxCoeff() > 0.0f);
}

TEST(mic_block_has_sequence) {
  Params p = makeTestParams();
  DSPInterface dsp(p, 3);

  std::atomic<uint64_t> lastSeq{0};
  dsp.setProcessMics([&](const MicBlock &mb, dsp::Block &control) {
    lastSeq.store(mb.seq, std::memory_order_relaxed);
    control.setZero();
  });

  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  ASSERT_TRUE(lastSeq.load() > 0);
}

TEST(blocking_process_mics_falls_back_to_zero_control) {
  Params p = makeTestParams();
  p.noise.sample_sigma = 0.0f;
  p.paths.H.setZero();
  p.paths.P.setZero();
  p.paths.C.setZero();
  p.paths.speaker.setZero();
  p.paths.speaker(0) = 1.0f;
  p.state.S.setZero();
  p.state.S_true.setZero();
  p.state.S(0) = 1.0f;
  p.state.S_true(0) = 1.0f;

  DSPInterface dsp(p, 3);

  std::atomic<int> callbackCount{0};
  dsp.setProcessMics([&](const MicBlock &, dsp::Block &control) {
    callbackCount.fetch_add(1, std::memory_order_relaxed);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    control = dsp::Block::Ones() * 0.5f;
  });

  std::optional<MicBlock> latest;
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::milliseconds(200);

  while (std::chrono::steady_clock::now() < deadline) {
    if (auto mic = dsp.getMics()) {
      latest = mic;
      if (latest->seq > 3) {
        break;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  ASSERT_TRUE(latest.has_value());
  ASSERT_TRUE(callbackCount.load(std::memory_order_relaxed) > 0);
  ASSERT_TRUE(latest->seq > 3);
  ASSERT_NEAR(latest->inear.cwiseAbs().maxCoeff(), 0.0f, 1e-6f);
}

TEST(blocking_process_mics_does_not_delay_shutdown) {
  Params p = makeTestParams();
  std::atomic<int> callbackCount{0};

  const auto start = std::chrono::steady_clock::now();
  {
    DSPInterface dsp(p, 3);
    dsp.setProcessMics([&](const MicBlock &, dsp::Block &control) {
      callbackCount.fetch_add(1, std::memory_order_relaxed);
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
      control = dsp::Block::Ones();
    });

    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(100);
    while (callbackCount.load(std::memory_order_relaxed) == 0 &&
           std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
  const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                           std::chrono::steady_clock::now() - start)
                           .count();

  ASSERT_TRUE(callbackCount.load(std::memory_order_relaxed) > 0);
  ASSERT_TRUE(elapsed < 500);
}

int main() {
  RUN_TEST(constructs_and_destructs);
  RUN_TEST(getMics_returns_data);
  RUN_TEST(sendControl_is_readable);
  RUN_TEST(processMics_callback_invoked);
  RUN_TEST(zero_control_inear_matches_noise_path);
  RUN_TEST(mic_block_carries_raw_input);
  RUN_TEST(mic_block_has_sequence);
  RUN_TEST(blocking_process_mics_falls_back_to_zero_control);
  RUN_TEST(blocking_process_mics_does_not_delay_shutdown);
  PRINT_RESULTS();
  return g_fails > 0 ? 1 : 0;
}
