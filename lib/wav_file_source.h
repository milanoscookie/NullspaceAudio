#pragma once

#include "audio_source.h"

#include <atomic>
#include <fstream>
#include <thread>
#include <vector>

class WavFileSource : public AudioSource {
public:
  struct Config {
    std::string inputPath;
    std::string outputPath;
    bool loop = false;
  };

  explicit WavFileSource(const Config &config);
  ~WavFileSource() override;

  // RT-unsafe.
  // Loads the WAV file, opens optional output, and stores the callback.
  void open(AudioCallback callback) override;

  // RT-unsafe.
  // Starts the streaming thread.
  void start() override;

  // RT-unsafe.
  // Stops the streaming thread and may block until it exits.
  void stop() override;

  // RT-unsafe.
  // Stops streaming and releases file-backed resources.
  void close() override;

  // RT-safe.
  bool isRunning() const override { return running_.load(); }

  // RT-safe.
  int getSampleRate() const override { return sampleRate_; }

private:
  void processThread();
  bool readWavFile();
  bool readBlock(dsp::Block &block);
  void writeWavHeader();
  void writeBlock(const dsp::Block &block);
  void finalizeWavOutput();

  Config config_;
  AudioCallback callback_;

  std::ofstream outputFile_;

  std::atomic<bool> running_{false};
  std::thread processThread_;

  int sampleRate_ = dsp::SAMPLE_RATE;
  int numChannels_ = 1;
  int bitsPerSample_ = 16;
  size_t totalSamples_ = 0;

  std::vector<float> audioBuffer_;
  size_t currentSample_ = 0;

  size_t samplesWritten_ = 0;
};
