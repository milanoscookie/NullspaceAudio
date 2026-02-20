#pragma once

#include "audio_source.h"
#include <atomic>
#include <fstream>
#include <memory>
#include <thread>
#include <vector>

/**
 * @brief WAV file audio source - pre-buffers entire file in heap memory
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
  bool readWavFile();
  bool readBlock(Block &block);
  void writeWavHeader();
  void writeBlock(const Block &block);
  void finalizeWavOutput();

  Config config_;
  AudioCallback callback_;

  std::ofstream outputFile_;

  std::atomic<bool> running_{false};
  std::thread processThread_;

  // WAV format info
  int sampleRate_ = dsp::SAMPLE_RATE;
  int numChannels_ = 1;
  int bitsPerSample_ = 16;
  size_t totalSamples_ = 0;

  // Pre-buffered audio data (float samples)
  std::vector<float> audioBuffer_;
  size_t currentSample_ = 0;

  // Output tracking
  size_t samplesWritten_ = 0;
};
