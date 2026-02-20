#pragma once

#include "dsp_config.h"
#include <Eigen/Dense>
#include <functional>
#include <memory>
#include <string>

using Block = Eigen::Matrix<float, dsp::BLOCK_SIZE, 1>;

/**
 * @brief Abstract interface for audio sources (WAV file, etc.)
 */
class AudioSource {
public:
  using AudioCallback = std::function<void(const Block &input, Block &output)>;

  virtual ~AudioSource() = default;

  /**
   * @brief Open the audio source with a callback
   * @param callback Function called for each audio block
   */
  virtual void open(AudioCallback callback) = 0;

  /**
   * @brief Start streaming audio
   */
  virtual void start() = 0;

  /**
   * @brief Stop streaming audio
   */
  virtual void stop() = 0;

  /**
   * @brief Close and release resources
   */
  virtual void close() = 0;

  /**
   * @brief Check if source is running
   */
  virtual bool isRunning() const = 0;

  /**
   * @brief Get sample rate
   */
  virtual int getSampleRate() const = 0;
};

/**
 * @brief Factory to create audio sources
 */
class AudioSourceFactory {
public:
  enum class Type { WavFile };

  struct Config {
    Type type = Type::WavFile;

    // WAV file options
    std::string inputWavPath;
    std::string outputWavPath;
    bool loop = false; // Loop WAV file when it ends
  };

  static std::unique_ptr<AudioSource> create(const Config &config);
};
