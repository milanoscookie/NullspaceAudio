#include "anc.h"
#include "dsp_interface.h"
#include "wav_writer.h"
#include <chrono>
#include <iostream>
#include <mutex>
#include <print>
#include <string>
#include <vector>
int main(int argc, char *argv[]) {
  try {
    // Help message
    if (argc > 1 &&
        (std::string(argv[1]) == "-h" || std::string(argv[1]) == "--help")) {
      std::cout << "Usage: " << argv[0] << " [input.wav] [output_prefix]"
                << std::endl;
      std::cout << "  input.wav      : Input WAV file (default: input.wav)"
                << std::endl;
      std::cout
          << "  output_prefix  : Prefix for output files (default: output)"
          << std::endl;
      std::cout
          << "Output files: <prefix>_desired_audio.wav, <prefix>_outside_mic.wav, <prefix>_inear_mic.wav"
          << std::endl;
      return 0;
    }

    // Parse command-line arguments
    std::string inputWavFile = "input.wav"; // Default input file
    std::string outputPrefix = "output";    // Default output prefix

    if (argc > 1) {
      inputWavFile = argv[1];
    }
    if (argc > 2) {
      outputPrefix = argv[2];
    }

    std::cout << "Using input WAV file: " << inputWavFile << std::endl;
    std::cout << "Output file prefix: " << outputPrefix << std::endl;

    AudioSourceFactory::Config audioConfig;
    audioConfig.inputWavPath = inputWavFile;

    // Create WAV writers for the raw WAV reference and simulated microphones
    std::string desiredAudioFile = outputPrefix + "_desired_audio.wav";
    std::string outsideFile = outputPrefix + "_outside_mic.wav";
    std::string inearFile = outputPrefix + "_inear_mic.wav";
    WavWriter wavWriterDesiredAudio(desiredAudioFile);
    WavWriter wavWriterOutside(outsideFile);
    WavWriter wavWriterInear(inearFile);
    std::vector<float> desiredAudioSamples;
    std::vector<float> outsideSamples;
    std::vector<float> inearSamples;
    std::mutex sampleMutex;

    {
      anc::init();

      // Create DSP interface with n block of system latency
      DSPInterface dspInterface(audioConfig, anc::systemLatencyBlocks);

      // Set up the microphone processing function
      dspInterface.setProcessMics([&](const MicBlock &micBlock,
                                      dsp::Block &control) {
        anc::step(micBlock, control);

        std::lock_guard<std::mutex> lk(sampleMutex);
        desiredAudioSamples.insert(desiredAudioSamples.end(),
                                   micBlock.desiredAudio.data(),
                                   micBlock.desiredAudio.data() + dsp::BLOCK_SIZE);
        outsideSamples.insert(outsideSamples.end(), micBlock.outside.data(),
                              micBlock.outside.data() + dsp::BLOCK_SIZE);
        inearSamples.insert(inearSamples.end(), micBlock.inear.data(),
                            micBlock.inear.data() + dsp::BLOCK_SIZE);
      });

      // Process microphone data until the WAV file is complete
      int blockCount = 0;
      auto startTime = std::chrono::steady_clock::now();
      auto lastReportTime = startTime;

      std::cout << "Starting audio processing..." << std::endl;

      // Continue processing while audio source is running or we still have
      // buffered data
      while (dspInterface.isAudioSourceRunning() || blockCount < 10) {
        // Get the next block of microphone data
        auto micBlock = dspInterface.getMics();

        if (micBlock) {
          blockCount++;

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
            // Audio source finished but we might have buffered data
            break;
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

    if (!wavWriterDesiredAudio.open() || !wavWriterOutside.open() ||
        !wavWriterInear.open()) {
      std::cerr << "Failed to open WAV files for writing" << std::endl;
      return 1;
    }

    wavWriterDesiredAudio.writeSamples(desiredAudioSamples);
    wavWriterOutside.writeSamples(outsideSamples);
    wavWriterInear.writeSamples(inearSamples);

  } catch (const std::exception &e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return 1;
  }

  return 0;
}
