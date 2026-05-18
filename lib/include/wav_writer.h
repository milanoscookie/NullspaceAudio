#pragma once

#include "dsp_config.h"

#include <fstream>
#include <string>
#include <vector>

struct WavWriterConfig {
  int sampleRate = dsp::SAMPLE_RATE;
  int numChannels = 1;
  int bitsPerSample = 16;
};

class WavWriter {
public:
  using Config = WavWriterConfig;

  explicit WavWriter(const std::string &path, const Config &config = {});
  ~WavWriter();

  WavWriter(const WavWriter &) = delete;
  WavWriter &operator=(const WavWriter &) = delete;

  // RT-unsafe.
  // Opens the destination file and writes a placeholder header.
  bool open();

  // RT-unsafe.
  // Writes one DSP block to the file.
  void writeBlock(const dsp::Block &block);

  // RT-unsafe.
  // Writes arbitrary floating-point samples.
  void writeSamples(const float *samples, size_t count);

  // RT-unsafe.
  // Writes a contiguous vector of floating-point samples.
  void writeSamples(const std::vector<float> &samples);

  // RT-unsafe.
  // Finalizes the header and closes the file.
  void close();

  // RT-safe.
  bool isOpen() const { return file_.is_open(); }

  // RT-safe.
  size_t getSamplesWritten() const { return samplesWritten_; }

  // RT-safe.
  float getDurationSeconds() const {
    return static_cast<float>(samplesWritten_) / config_.sampleRate;
  }

private:
  void writeHeader();
  void finalizeHeader();

  std::string path_;
  Config config_;
  std::ofstream file_;
  size_t samplesWritten_ = 0;
  size_t dataChunkPos_ = 0;
};
