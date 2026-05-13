#pragma once

#include "dsp_config.h"

#include <functional>
#include <memory>
#include <string>

class AudioSource {
public:
  using AudioCallback =
      std::function<void(const dsp::Block &input, dsp::Block &output)>;

  virtual ~AudioSource() = default;

  // RT-unsafe.
  // Registers the processing callback and acquires source resources.
  virtual void open(AudioCallback callback) = 0;

  // RT-unsafe.
  // Starts background streaming.
  virtual void start() = 0;

  // RT-unsafe.
  // Stops background streaming and may wait for shutdown.
  virtual void stop() = 0;

  // RT-unsafe.
  // Releases source resources.
  virtual void close() = 0;

  // RT-safe.
  // Returns whether streaming is active.
  virtual bool isRunning() const = 0;

  // RT-safe.
  // Returns the configured sample rate.
  virtual int getSampleRate() const = 0;
};

class AudioSourceFactory {
public:
  enum class Type { WavFile };

  struct Config {
    Type type = Type::WavFile;

    std::string inputWavPath;
    std::string outputWavPath;
    bool loop = false;
  };

  // RT-unsafe.
  // Allocates and configures a concrete audio source.
  static std::unique_ptr<AudioSource> create(const Config &config);
};
