#include "dsp_interface.h"
#include "audio_source.h"
#include "portaudio_stream.h"
#include "wav_writer.h"
#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>
#include <atomic>

// ──────────────────────────────────────────────────────────────
// FxLMS adaptive filter state  (lives outside the lambda so we
// can inspect it from the main thread for logging)
// ──────────────────────────────────────────────────────────────
struct FxLMS {
  static constexpr int N = dsp::IR_SIZE; // adaptive filter length

  // Adaptive weight vector  w
  IRBlock w = IRBlock::Zero();

  // Reference-signal history  x[n], x[n-1], ..., x[n-N+1]
  // Stored newest-first so w.dot(xHist) = FIR output
  IRBlock xHist = IRBlock::Zero();

  // Filtered-x history  x'[n] = S_hat * x[n]
  IRBlock xfHist = IRBlock::Zero();

  // S-hat FIR system for filtering the reference
  LinearSystem<dsp::IR_SIZE> sHatSystem;

  float mu = 0.0001f; // step size

  // ---- convenience ---------------------------------------------------
  void init(const IRBlock &sHat, float stepSize) {
    sHatSystem.setImpulseResponse(sHat);
    mu = stepSize;
    w.setZero();
    xHist.setZero();
    xfHist.setZero();
  }

  // Process one block: given outside & inear, return control u
  void processBlock(const Block &outside, const Block &inear, Block &u) {
    Block xfBlock = Block::Zero();
    // Filter the reference (outside) through S_hat to get x'
    sHatSystem.step(outside, xfBlock);

    for (int n = 0; n < dsp::BLOCK_SIZE; ++n) {
      // Shift reference history (newest at index 0)
      for (int k = N - 1; k > 0; --k)
        xHist(k) = xHist(k - 1);
      xHist(0) = outside(n);

      // Shift filtered-x history
      for (int k = N - 1; k > 0; --k)
        xfHist(k) = xfHist(k - 1);
      xfHist(0) = xfBlock(n);

      // FIR output:  y[n] = w^T x
      float y = w.dot(xHist);
      u(n) = -y; // anti-noise (negative to cancel)

      // Error signal is the in-ear mic sample
      float e = inear(n);

      // Normalized LMS weight update:  w -= mu / (x'^T x' + eps) * e * x'
      const float xfPow = xfHist.squaredNorm() + 1e-6f;
      w -= (mu / xfPow) * e * xfHist;
    }
  }
};

// ──────────────────────────────────────────────────────────────
// Test: FxLMS ANC with PortAudio
// ──────────────────────────────────────────────────────────────
void testFxLMS_PortAudio() {
  std::cout << "\n=== FxLMS ANC Test (PortAudio) ===" << std::endl;
  std::cout << PortAudioStream::listDevices() << std::endl;

  Params params;
  // Simple delay-only paths for initial testing
  params.paths.H(0) = 1.0f;  // noise -> outside (direct)
  params.paths.P(0) = 0.8f;  // noise -> in-ear (slightly attenuated)
  params.paths.C(0) = 0.1f;  // speaker -> outside mic (leakage)
  params.paths.speaker(0) = 1.0f;
  params.state.S(0) = 0.9f;  // speaker -> in-ear (secondary path)
  params.state.S_true = params.state.S;

  params.audioConfig.type = AudioSourceFactory::Type::PortAudio;
  params.audioConfig.inputChannels = 1;
  params.audioConfig.outputChannels = 1;

  // S-hat = our estimate of the secondary path (start with true value)
  IRBlock sHat = params.state.S;

  int systemLatency = 3; // ms

  FxLMS fxlms;
  fxlms.init(sHat, 0.0001f);

  std::atomic<float> errPower{0.0f};
  std::atomic<int> blockCount{0};

  DSPInterface dsp(params, systemLatency);
  dsp.setProcessMics([&](const MicBlock &mb, Block &control) {
    fxlms.processBlock(mb.outside, mb.inear, control);

    // Track error power (in-ear RMS)
    float rms = std::sqrt(mb.inear.squaredNorm() / dsp::BLOCK_SIZE);
    errPower.store(rms, std::memory_order_relaxed);
    blockCount.fetch_add(1, std::memory_order_relaxed);
  });

  std::cout << "Running FxLMS ANC for 10 seconds..." << std::endl;
  auto start = Clock::now();
  int lastReport = 0;

  while (Clock::now() - start < std::chrono::seconds(10)) {
    int bc = blockCount.load(std::memory_order_relaxed);
    int elapsed = static_cast<int>(
        std::chrono::duration_cast<std::chrono::seconds>(Clock::now() - start)
            .count());
    if (elapsed > lastReport) {
      lastReport = elapsed;
      float ep = errPower.load(std::memory_order_relaxed);
      float wNorm = std::sqrt(fxlms.w.squaredNorm());
      std::cout << "  t=" << elapsed << "s  blocks=" << bc
                << "  inear_rms=" << ep
                << "  ||w||=" << wNorm << std::endl;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  std::cout << "\nFinal ||w|| = " << std::sqrt(fxlms.w.squaredNorm())
            << std::endl;
  std::cout << "Final inear RMS = "
            << errPower.load(std::memory_order_relaxed) << std::endl;
}

// ──────────────────────────────────────────────────────────────
// Test: FxLMS ANC with WAV file
// ──────────────────────────────────────────────────────────────
void testFxLMS_Wav(const std::string &inputPath,
                   const std::string &outputPath) {
  std::cout << "\n=== FxLMS ANC Test (WAV) ===" << std::endl;
  std::cout << "Input:  " << inputPath << std::endl;
  std::cout << "Output: " << outputPath << std::endl;

  Params params;
  params.paths.H(0) = 1.0f;
  params.paths.P(0) = 0.8f;
  params.paths.C(0) = 0.1f;
  params.paths.speaker(0) = 1.0f;
  params.state.S(0) = 0.9f;
  params.state.S_true = params.state.S;

  params.audioConfig.type = AudioSourceFactory::Type::WavFile;
  params.audioConfig.inputWavPath = inputPath;
  params.audioConfig.outputWavPath = outputPath;
  params.audioConfig.loop = false;

  // Disable synthetic noise for WAV test — the WAV IS the noise
  params.noise.sample_sigma = 0.0f;
  params.noise.sigma_fc_hz = 0.0f;
  params.noise.fc_mean_hz = 0.0f;

  IRBlock sHat = params.state.S;
  int systemLatency = 3;

  FxLMS fxlms;
  fxlms.init(sHat, 0.0001f);

  std::atomic<float> errPower{0.0f};
  std::atomic<float> initPower{-1.0f};
  std::atomic<int> blockCount{0};

  DSPInterface dsp(params, systemLatency);
  dsp.setProcessMics([&](const MicBlock &mb, Block &control) {
    fxlms.processBlock(mb.outside, mb.inear, control);

    float rms = std::sqrt(mb.inear.squaredNorm() / dsp::BLOCK_SIZE);
    errPower.store(rms, std::memory_order_relaxed);

    // Capture initial error power (before adaptation takes effect)
    float expected = -1.0f;
    initPower.compare_exchange_strong(expected, rms,
                                      std::memory_order_relaxed);

    blockCount.fetch_add(1, std::memory_order_relaxed);
  });

  std::cout << "Processing..." << std::endl;
  auto start = Clock::now();
  int lastReport = 0;
  int prevBlocks = 0;

  while (Clock::now() - start < std::chrono::seconds(60)) {
    int bc = blockCount.load(std::memory_order_relaxed);
    int elapsed = static_cast<int>(
        std::chrono::duration_cast<std::chrono::seconds>(Clock::now() - start)
            .count());

    if (elapsed > lastReport && elapsed > 0) {
      lastReport = elapsed;
      float ep = errPower.load(std::memory_order_relaxed);
      float ip = initPower.load(std::memory_order_relaxed);
      float reduction_dB =
          (ip > 1e-12f && ep > 1e-12f) ? 20.0f * std::log10(ep / ip) : 0.0f;
      std::cout << "  t=" << elapsed << "s  blocks=" << bc
                << "  inear_rms=" << ep
                << "  reduction=" << reduction_dB << " dB" << std::endl;
    }

    // Detect end of file: no new blocks for a while
    if (bc > 0 && bc == prevBlocks) {
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
      if (blockCount.load(std::memory_order_relaxed) == bc)
        break;
    }
    prevBlocks = bc;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  float ip = initPower.load(std::memory_order_relaxed);
  float ep = errPower.load(std::memory_order_relaxed);
  float reduction_dB =
      (ip > 1e-12f && ep > 1e-12f) ? 20.0f * std::log10(ep / ip) : 0.0f;
  std::cout << "\n=== FxLMS Results ===" << std::endl;
  std::cout << "Blocks processed: " << blockCount.load() << std::endl;
  std::cout << "Initial inear RMS: " << ip << std::endl;
  std::cout << "Final   inear RMS: " << ep << std::endl;
  std::cout << "Noise reduction:   " << reduction_dB << " dB" << std::endl;
  std::cout << "||w|| = " << std::sqrt(fxlms.w.squaredNorm()) << std::endl;
}

// ──────────────────────────────────────────────────────────────
// Write mic signals to WAV files with FxLMS ANC active
// ──────────────────────────────────────────────────────────────
void writeMics(const std::string &prefix, int durationSeconds) {
  const std::string outsidePath = prefix + "_outside.wav";
  const std::string inearPath = prefix + "_inear.wav";

  std::cout << "\n=== Write Mics (FxLMS ANC active) ===" << std::endl;
  std::cout << "Duration: " << durationSeconds << "s" << std::endl;
  std::cout << "Outside mic -> " << outsidePath << std::endl;
  std::cout << "In-ear mic  -> " << inearPath << std::endl;

  WavWriter::Config wCfg;
  wCfg.sampleRate = dsp::SAMPLE_RATE;
  wCfg.numChannels = 1;
  wCfg.bitsPerSample = 16;

  WavWriter outsideWriter(outsidePath, wCfg);
  WavWriter inearWriter(inearPath, wCfg);
  if (!outsideWriter.open() || !inearWriter.open()) {
    std::cerr << "Failed to open output WAV files" << std::endl;
    return;
  }

  Params params;
  params.paths.H(0) = 1.0f;
  params.paths.P(0) = 0.8f;
  params.paths.C(0) = 0.1f;
  params.paths.speaker(0) = 1.0f;
  params.state.S(0) = 0.9f;
  params.state.S_true = params.state.S;

  params.audioConfig.type = AudioSourceFactory::Type::PortAudio;
  params.audioConfig.inputChannels = 1;
  params.audioConfig.outputChannels = 1;

  IRBlock sHat = params.state.S;
  int systemLatency = 3;

  FxLMS fxlms;
  fxlms.init(sHat, 0.0001f);

  std::atomic<float> errPower{0.0f};
  std::atomic<float> initPower{-1.0f};
  std::atomic<int> blockCount{0};

  try {
    DSPInterface dsp(params, systemLatency);
    dsp.setProcessMics([&](const MicBlock &mb, Block &control) {
      // Bypass ANC — zero control, just observe mics
      control.setZero();

      float rms = std::sqrt(mb.inear.squaredNorm() / dsp::BLOCK_SIZE);
      errPower.store(rms, std::memory_order_relaxed);

      float expected = -1.0f;
      initPower.compare_exchange_strong(expected, rms,
                                        std::memory_order_relaxed);
      blockCount.fetch_add(1, std::memory_order_relaxed);
    });

    std::cout << "Recording mics with FxLMS ANC..." << std::endl;

    auto startTime = Clock::now();
    int blocksWritten = 0;
    const int blocksPerSecond = dsp::SAMPLE_RATE / dsp::BLOCK_SIZE;

    while (Clock::now() - startTime <
           std::chrono::seconds(durationSeconds)) {
      auto mic = dsp.getMics();
      if (mic.has_value()) {
        outsideWriter.writeBlock(mic->outside);
        inearWriter.writeBlock(mic->inear);
        blocksWritten++;

        if (blocksWritten % blocksPerSecond == 0) {
          int elapsed = blocksWritten / blocksPerSecond;
          float ip = initPower.load(std::memory_order_relaxed);
          float ep = errPower.load(std::memory_order_relaxed);
          float reduction_dB =
              (ip > 1e-12f && ep > 1e-12f)
                  ? 20.0f * std::log10(ep / ip)
                  : 0.0f;
          std::cout << "  " << elapsed << "s / " << durationSeconds
                    << "s  inear_rms=" << ep
                    << "  reduction=" << reduction_dB << " dB"
                    << std::endl;
        }
      }
      std::this_thread::sleep_for(std::chrono::microseconds(500));
    }

    outsideWriter.close();
    inearWriter.close();

    float ip = initPower.load(std::memory_order_relaxed);
    float ep = errPower.load(std::memory_order_relaxed);
    float reduction_dB =
        (ip > 1e-12f && ep > 1e-12f) ? 20.0f * std::log10(ep / ip) : 0.0f;
    float durationSec =
        blocksWritten * dsp::BLOCK_SIZE /
        static_cast<float>(dsp::SAMPLE_RATE);
    std::cout << "\n=== Done ===" << std::endl;
    std::cout << "Blocks written: " << blocksWritten << std::endl;
    std::cout << "Duration: " << durationSec << "s" << std::endl;
    std::cout << "Noise reduction: " << reduction_dB << " dB" << std::endl;
    std::cout << "||w|| = " << std::sqrt(fxlms.w.squaredNorm()) << std::endl;
    std::cout << "Files: " << outsidePath << ", " << inearPath
              << std::endl;

  } catch (const std::exception &e) {
    std::cerr << "Error: " << e.what() << std::endl;
  }
}

void printUsage(const char* progName) {
  std::cout << "Usage: " << progName << " [options]" << std::endl;
  std::cout << std::endl;
  std::cout << "Options:" << std::endl;
  std::cout << "  --fxlms               Run FxLMS ANC with PortAudio (live mic)" << std::endl;
  std::cout << "  --fxlms-wav <in> [out] Run FxLMS ANC on a WAV file" << std::endl;
  std::cout << "  --write-mics <prefix> <s> Write noisy mics to WAV (no ANC)" << std::endl;
  std::cout << "  --portaudio            Passthrough test with PortAudio" << std::endl;
  std::cout << "  --record <output> <s>  Record from microphone to WAV" << std::endl;
  std::cout << "  --help                 Show this help" << std::endl;
  std::cout << std::endl;
  std::cout << "Examples:" << std::endl;
  std::cout << "  " << progName << " --fxlms" << std::endl;
  std::cout << "  " << progName << " --fxlms-wav noise.wav cancelled.wav" << std::endl;
  std::cout << "  " << progName << " --write-mics test 5" << std::endl;
  std::cout << "  " << progName << " --record recording.wav 10" << std::endl;
}

void recordToWav(const std::string& outputPath, int durationSeconds) {
  std::cout << "\n=== Recording to WAV ===" << std::endl;
  std::cout << "Output: " << outputPath << std::endl;
  std::cout << "Duration: " << durationSeconds << " seconds" << std::endl;

  // Setup WavWriter
  WavWriter::Config writerConfig;
  writerConfig.sampleRate = dsp::SAMPLE_RATE;
  writerConfig.numChannels = 1;
  writerConfig.bitsPerSample = 16;  // 16-bit PCM

  WavWriter writer(outputPath, writerConfig);
  if (!writer.open()) {
    std::cerr << "Failed to open output file" << std::endl;
    return;
  }

  // Setup DSPInterface with PortAudio
  Params params;
  params.paths.H(0) = 1.0f;
  params.paths.P(0) = 1.0f;
  params.paths.C(0) = 1.0f;
  params.paths.speaker(0) = 1.0f;
  params.state.S(0) = 1.0f;
  
  params.audioConfig.type = AudioSourceFactory::Type::PortAudio;
  params.audioConfig.inputChannels = 1;
  params.audioConfig.outputChannels = 1;

  int systemLatency = 3;

  try {
    std::cout << "Creating DSPInterface..." << std::endl;
    DSPInterface dsp(params, systemLatency);
    dsp.setProcessMics([](const MicBlock &mb, Block &control) {
      (void)mb;
      control.setZero();
    });

    std::cout << "Recording... (speak into microphone)" << std::endl;

    auto startTime = std::chrono::steady_clock::now();
    int blocksReceived = 0;
    float maxAmplitude = 0.0f;

    while (std::chrono::steady_clock::now() - startTime < std::chrono::seconds(durationSeconds)) {
      auto micData = dsp.getMics();
      if (micData.has_value()) {
        blocksReceived++;
        
        // Write to WAV file
        writer.writeBlock(micData->outside);
        
        // Track max amplitude
        float blockMax = micData->outside.cwiseAbs().maxCoeff();
        maxAmplitude = std::max(maxAmplitude, blockMax);

        // Progress every second
        int blocksPerSecond = dsp::SAMPLE_RATE / dsp::BLOCK_SIZE;
        if (blocksReceived % blocksPerSecond == 0) {
          int elapsed = blocksReceived / blocksPerSecond;
          std::cout << "  " << elapsed << "s / " << durationSeconds << "s recorded" << std::endl;
        }
      }
      std::this_thread::sleep_for(std::chrono::microseconds(500));
    }

    writer.close();

    std::cout << "\n=== Recording Complete ===" << std::endl;
    std::cout << "Blocks recorded: " << blocksReceived << std::endl;
    std::cout << "Max amplitude: " << maxAmplitude << std::endl;

  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << std::endl;
  }
}

int main(int argc, char* argv[]) {
  std::cout << "=== DSP Interface Test ===" << std::endl;
  std::cout << "Block size: " << dsp::BLOCK_SIZE << " samples" << std::endl;
  std::cout << "Sample rate: " << dsp::SAMPLE_RATE << " Hz" << std::endl;
  std::cout << "Block latency: " << dsp::BLOCK_LATENCY_MS << " ms" << std::endl;

  if (argc < 2) {
    printUsage(argv[0]);
    return 1;
  }

  std::string mode = argv[1];

  if (mode == "--help" || mode == "-h") {
    printUsage(argv[0]);
    return 0;
  }

  if (mode == "--fxlms") {
    testFxLMS_PortAudio();
  } else if (mode == "--fxlms-wav") {
    if (argc < 3) {
      std::cerr << "Error: --fxlms-wav requires input file path" << std::endl;
      printUsage(argv[0]);
      return 1;
    }
    std::string inputPath = argv[2];
    std::string outputPath = (argc >= 4) ? argv[3] : "";
    testFxLMS_Wav(inputPath, outputPath);
  } else if (mode == "--portaudio") {
    testFxLMS_PortAudio(); // default to FxLMS
  } else if (mode == "--write-mics") {
    if (argc < 4) {
      std::cerr << "Error: --write-mics requires <prefix> <seconds>" << std::endl;
      printUsage(argv[0]);
      return 1;
    }
    std::string prefix = argv[2];
    int duration = std::stoi(argv[3]);
    writeMics(prefix, duration);
  } else if (mode == "--record") {
    if (argc < 4) {
      std::cerr << "Error: --record requires output file path and duration" << std::endl;
      printUsage(argv[0]);
      return 1;
    }
    std::string outputPath = argv[2];
    int duration = std::stoi(argv[3]);
    recordToWav(outputPath, duration);
  } else {
    std::cerr << "Unknown option: " << mode << std::endl;
    printUsage(argv[0]);
    return 1;
  }

  return 0;
}
