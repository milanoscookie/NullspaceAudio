#include "anc.h"
#include "dsp_interface.h"
#include "wav_writer.h"
#include <chrono>
#include <atomic>
#include <iostream>
#include <memory>
#include <string>
int main(int argc, char *argv[]) {
  try {
    // Help message
    if (argc > 1 &&
        (std::string(argv[1]) == "-h" || std::string(argv[1]) == "--help")) {
      std::cout << "Usage: " << argv[0] << " <input.wav> [output_prefix]\n"
                << "  input.wav      : Input WAV file\n"
                << "  output_prefix  : Prefix for output files (default: output)\n"
                << "Output files: <prefix>_raw_input.wav, <prefix>_outside_mic.wav, <prefix>_inear_mic.wav\n";
      return 0;
    }

    if (argc < 2) {
      std::cerr << "Error: missing input WAV file\n";
      std::cerr << "Usage: " << argv[0] << " <input.wav> [output_prefix]"
                << std::endl;
      return 1;
    }

    // Parse command-line arguments
    std::string inputWavFile = argv[1];
    std::string outputPrefix = "output";    // Default output prefix

    if (argc > 2) {
      outputPrefix = argv[2];
    }

    std::cout << "Using input WAV file: " << inputWavFile << std::endl;
    std::cout << "Output file prefix: " << outputPrefix << std::endl;

    AudioSourceFactory::Config audioConfig;
    audioConfig.inputWavPath = inputWavFile;

    // Create WAV writers for the raw WAV reference and simulated microphones
    std::string rawInputFile = outputPrefix + "_raw_input.wav";
    std::string outsideFile = outputPrefix + "_outside_mic.wav";
    std::string inearFile = outputPrefix + "_inear_mic.wav";

    struct CaptureState {
      std::atomic<int> observedBlocks{0};
      std::shared_ptr<WavWriter> rawInput;
      std::shared_ptr<WavWriter> outside;
      std::shared_ptr<WavWriter> inear;
    };

    auto captureState = std::make_shared<CaptureState>();
    captureState->rawInput = std::make_shared<WavWriter>(rawInputFile);
    captureState->outside = std::make_shared<WavWriter>(outsideFile);
    captureState->inear = std::make_shared<WavWriter>(inearFile);

    if (!captureState->rawInput->open() || !captureState->outside->open() ||
        !captureState->inear->open()) {
      std::cerr << "Failed to open WAV files for writing" << std::endl;
      return 1;
    }

    {
      anc::init();

      // Create DSP interface with configured simulator latency
      DSPInterface dspInterface(audioConfig, dsp::SYSTEM_LATENCY_BLOCKS);

      dspInterface.setObserveMics([captureState](const MicBlock &micBlock) {
        captureState->observedBlocks.fetch_add(1, std::memory_order_relaxed);
        captureState->rawInput->writeBlock(micBlock.rawInput);
        captureState->outside->writeBlock(micBlock.outside);
        captureState->inear->writeBlock(micBlock.inear);
      });

      // Set up the microphone processing function after observers are ready.
      dspInterface.setProcessMics(anc::step);

      // Process microphone data until the WAV file is complete
      int blockCount = 0;
      auto startTime = std::chrono::steady_clock::now();
      auto lastReportTime = startTime;

      std::cout << "Starting audio processing..." << std::endl;

      // Continue processing while audio source is running or we still have
      // buffered data
      int emptyPollsAfterStop = 0;
      constexpr int kDrainPollLimit = 2000;
      while (dspInterface.isAudioSourceRunning() ||
             emptyPollsAfterStop < kDrainPollLimit) {
        const int newBlockCount =
            captureState->observedBlocks.load(std::memory_order_relaxed);
        if (newBlockCount > blockCount) {
          blockCount = newBlockCount;
          emptyPollsAfterStop = 0;

          // Report progress based on actual elapsed time
          auto now = std::chrono::steady_clock::now();
          auto elapsedMs =
              std::chrono::duration_cast<std::chrono::milliseconds>(
                  now - lastReportTime)
                  .count();

          // Report every ~1000ms of actual elapsed time
          if (elapsedMs >= 1000) {
            auto totalElapsedSecs =
                std::chrono::duration_cast<std::chrono::seconds>(now - startTime)
                    .count();
            std::cout << "Processed " << totalElapsedSecs << " second(s) - "
                      << blockCount << " blocks" << std::endl;
            lastReportTime = now;
          }
        } else {
          // No data available yet, wait a bit and try again
          if (dspInterface.isAudioSourceRunning()) {
            std::this_thread::sleep_for(std::chrono::microseconds(
                10)); // Very short sleep to avoid blocking
          } else {
            ++emptyPollsAfterStop;
            std::this_thread::sleep_for(std::chrono::microseconds(50));
          }
        }
      }

      auto endTime = std::chrono::steady_clock::now();
      auto totalElapsedMs =
          std::chrono::duration_cast<std::chrono::milliseconds>(endTime -
                                                                startTime)
              .count();
      std::cout << "Audio processing complete. Total blocks processed: "
                << blockCount << " in " << totalElapsedMs << "ms"
                << std::endl;
    }

  } catch (const std::exception &e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return 1;
  }

  return 0;
}
