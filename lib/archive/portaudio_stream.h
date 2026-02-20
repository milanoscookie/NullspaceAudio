#pragma once

#include "dsp_config.h"
#include <portaudio.h>

#include <Eigen/Dense>
#include <atomic>
#include <functional>
#include <string>

using Block = Eigen::Matrix<float, dsp::BLOCK_SIZE, 1>;

/**
 * @brief PortAudio wrapper for real-time audio I/O with block-based processing
 *
 * Reads and writes audio in BLOCK_SIZE sample chunks, matching the DSP latency.
 */
class PortAudioStream {
public:
  using AudioCallback = std::function<void(const Block &input, Block &output)>;

  struct Config {
    int inputDevice = -1;  // -1 for default
    int outputDevice = -1; // -1 for default
    int inputChannels = 1;
    int outputChannels = 1;
    double sampleRate = static_cast<double>(dsp::SAMPLE_RATE);
  };

  PortAudioStream();
  ~PortAudioStream();

  // Non-copyable
  PortAudioStream(const PortAudioStream &) = delete;
  PortAudioStream &operator=(const PortAudioStream &) = delete;

  /**
   * @brief Initialize and open the audio stream
   * @param config Stream configuration
   * @param callback Function called for each audio block
   */
  void open(const Config &config, AudioCallback callback);

  /**
   * @brief Start the audio stream
   */
  void start();

  /**
   * @brief Stop the audio stream
   */
  void stop();

  /**
   * @brief Close the audio stream and release resources
   */
  void close();

  /**
   * @brief Check if the stream is currently running
   */
  bool isRunning() const { return running_.load(); }

  /**
   * @brief Get the actual input latency in samples
   */
  int getInputLatencySamples() const;

  /**
   * @brief Get the actual output latency in samples
   */
  int getOutputLatencySamples() const;

  /**
   * @brief List available audio devices
   */
  static std::string listDevices();

private:
  static int paCallback(const void *inputBuffer, void *outputBuffer,
                        unsigned long framesPerBuffer,
                        const PaStreamCallbackTimeInfo *timeInfo,
                        PaStreamCallbackFlags statusFlags, void *userData);

  PaStream *stream_ = nullptr;
  AudioCallback callback_;
  std::atomic<bool> running_{false};
  bool initialized_ = false;

  // Temp buffers for Eigen interop
  Block inputBlock_;
  Block outputBlock_;
};
