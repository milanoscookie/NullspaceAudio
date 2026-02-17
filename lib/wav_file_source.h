#pragma once

#include "audio_source.h"
#include <atomic>
#include <fstream>
#include <thread>

/**
 * @brief WAV file audio source - reads from WAV file, optionally writes output
 */
class WavFileSource : public AudioSource {
public:
  struct Config {
    std::string inputPath;
    std::string outputPath; // Empty = no output
    bool loop = false;
  };

  explicit WavFileSource(const Config &config);
  ~WavFileSource() override;

  void open(AudioCallback callback) override;
  void start() override;
  void stop() override;
  void close() override;
  bool isRunning() const override { return running_.load(); }
  int getSampleRate() const override { return sampleRate_; }

private:
  void processThread();
  bool readWavHeader();
  bool readBlock(Block &block);
  void writeWavHeader();
  void writeBlock(const Block &block);
  void finalizeWavOutput();

  Config config_;
  AudioCallback callback_;

  std::ifstream inputFile_;
  std::ofstream outputFile_;

  std::atomic<bool> running_{false};
  std::thread processThread_;

  // WAV format info
  int sampleRate_ = dsp::SAMPLE_RATE;
  int numChannels_ = 1;
  int bitsPerSample_ = 16;
  size_t dataSize_ = 0;
  size_t dataStart_ = 0;
  size_t samplesRead_ = 0;
  size_t totalSamples_ = 0;

  // Output tracking
  size_t samplesWritten_ = 0;
};
