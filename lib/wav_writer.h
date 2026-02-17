#pragma once

#include "dsp_config.h"
#include <Eigen/Dense>
#include <fstream>
#include <string>
#include <vector>

using Block = Eigen::Matrix<float, dsp::BLOCK_SIZE, 1>;

/**
 * @brief Simple WAV file writer for exporting audio data
 *
 * Usage:
 *   WavWriter writer("output.wav");
 *   writer.open();
 *   writer.writeBlock(block1);
 *   writer.writeBlock(block2);
 *   writer.close();  // Finalizes header with correct size
 */

// Forward declare Config outside the class
struct WavWriterConfig {
  int sampleRate = dsp::SAMPLE_RATE;
  int numChannels = 1;
  int bitsPerSample = 16; // 16 or 32 (float)
};

class WavWriter {
public:
  using Config = WavWriterConfig;

  explicit WavWriter(const std::string &path, const Config &config = {});
  ~WavWriter();

  // Non-copyable
  WavWriter(const WavWriter &) = delete;
  WavWriter &operator=(const WavWriter &) = delete;

  /**
   * @brief Open the file for writing
   */
  bool open();

  /**
   * @brief Write a single block of samples
   */
  void writeBlock(const Block &block);

  /**
   * @brief Write arbitrary float samples
   */
  void writeSamples(const float *samples, size_t count);

  /**
   * @brief Write a vector of samples
   */
  void writeSamples(const std::vector<float> &samples);

  /**
   * @brief Close and finalize the file (updates header with correct size)
   */
  void close();

  /**
   * @brief Check if file is open
   */
  bool isOpen() const { return file_.is_open(); }

  /**
   * @brief Get number of samples written
   */
  size_t getSamplesWritten() const { return samplesWritten_; }

  /**
   * @brief Get duration in seconds
   */
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
