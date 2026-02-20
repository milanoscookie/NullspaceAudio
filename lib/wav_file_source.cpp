#include "wav_file_source.h"
#include <chrono>
#include <cstring>
#include <iostream>
#include <stdexcept>

WavFileSource::WavFileSource(const Config &config) : config_(config) {}

WavFileSource::~WavFileSource() { close(); }

void WavFileSource::open(AudioCallback callback) {
  callback_ = std::move(callback);

  // Read and buffer entire WAV file
  if (!readWavFile()) {
    throw std::runtime_error("Failed to read WAV file: " + config_.inputPath);
  }

  // Open output WAV file if specified
  if (!config_.outputPath.empty()) {
    outputFile_.open(config_.outputPath, std::ios::binary);
    if (!outputFile_.is_open()) {
      throw std::runtime_error("Failed to open output WAV file: " +
                               config_.outputPath);
    }
    writeWavHeader(); // Write placeholder header
  }

  std::cout << "WAV file loaded into memory: " << config_.inputPath
            << std::endl;
  std::cout << "  Sample rate: " << sampleRate_ << " Hz" << std::endl;
  std::cout << "  Channels: " << numChannels_ << std::endl;
  std::cout << "  Bits per sample: " << bitsPerSample_ << std::endl;
  std::cout << "  Total samples: " << totalSamples_ << std::endl;
  std::cout << "  Buffer size: " << (audioBuffer_.size() * sizeof(float) / 1024.0f / 1024.0f)
            << " MB" << std::endl;
  std::cout << "  Duration: "
            << (totalSamples_ / static_cast<float>(sampleRate_)) << " seconds"
            << std::endl;
}

void WavFileSource::start() {
  if (running_.load())
    return;

  running_.store(true);
  currentSample_ = 0;
  processThread_ = std::thread(&WavFileSource::processThread, this);
}

void WavFileSource::stop() {
  running_.store(false);
  if (processThread_.joinable()) {
    processThread_.join();
  }
}

void WavFileSource::close() {
  stop();

  if (outputFile_.is_open()) {
    finalizeWavOutput();
    outputFile_.close();
  }

  // Clear the pre-buffered audio
  audioBuffer_.clear();
  audioBuffer_.shrink_to_fit();
}

void WavFileSource::processThread() {
  const auto blockDuration =
      std::chrono::microseconds((dsp::BLOCK_SIZE * 1000000) / sampleRate_);

  Block inputBlock;
  Block outputBlock;

  while (running_.load()) {
    auto startTime = std::chrono::steady_clock::now();

    // Read a block from pre-buffered audio
    if (!readBlock(inputBlock)) {
      if (config_.loop) {
        // Loop back to start
        currentSample_ = 0;
        if (!readBlock(inputBlock)) {
          running_.store(false);
          break;
        }
      } else {
        // End of file
        running_.store(false);
        break;
      }
    }

    // Process through callback
    outputBlock.setZero();
    if (callback_) {
      callback_(inputBlock, outputBlock);
    }

    // Write output if configured
    if (outputFile_.is_open()) {
      writeBlock(outputBlock);
    }

    // Simulate real-time by sleeping
    // auto elapsed = std::chrono::steady_clock::now() - startTime;
    // auto sleepTime = blockDuration - elapsed;
    // if (sleepTime > std::chrono::microseconds(0)) {
    //   std::this_thread::sleep_for(sleepTime);
    // }
  }
}

bool WavFileSource::readWavFile() {
  std::ifstream file(config_.inputPath, std::ios::binary);
  if (!file.is_open()) {
    std::cerr << "Failed to open WAV file: " << config_.inputPath << std::endl;
    return false;
  }

  // Read and parse WAV header
  char buffer[44];
  file.read(buffer, 44);
  if (file.gcount() < 44) {
    std::cerr << "WAV file too small" << std::endl;
    return false;
  }

  // Check RIFF header
  if (std::strncmp(buffer, "RIFF", 4) != 0) {
    std::cerr << "Invalid RIFF header" << std::endl;
    return false;
  }
  if (std::strncmp(buffer + 8, "WAVE", 4) != 0) {
    std::cerr << "Invalid WAVE header" << std::endl;
    return false;
  }
  if (std::strncmp(buffer + 12, "fmt ", 4) != 0) {
    std::cerr << "Invalid fmt chunk" << std::endl;
    return false;
  }

  // Parse format chunk
  uint16_t audioFormat = *reinterpret_cast<uint16_t *>(buffer + 20);
  if (audioFormat != 1 && audioFormat != 3) { // 1 = PCM, 3 = IEEE float
    std::cerr << "Unsupported audio format: " << audioFormat << std::endl;
    return false;
  }

  numChannels_ = *reinterpret_cast<uint16_t *>(buffer + 22);
  sampleRate_ = *reinterpret_cast<uint32_t *>(buffer + 24);
  bitsPerSample_ = *reinterpret_cast<uint16_t *>(buffer + 34);

  // Find data chunk
  file.seekg(12);

  size_t dataSize = 0;
  while (file.good()) {
    char chunkId[4];
    uint32_t chunkSize;

    file.read(chunkId, 4);
    file.read(reinterpret_cast<char *>(&chunkSize), 4);

    if (std::strncmp(chunkId, "data", 4) == 0) {
      dataSize = chunkSize;
      break;
    }

    // Skip this chunk
    file.seekg(chunkSize, std::ios::cur);
  }

  if (dataSize == 0) {
    std::cerr << "No data chunk found" << std::endl;
    return false;
  }

  // Calculate total samples
  int bytesPerSample = bitsPerSample_ / 8;
  totalSamples_ = dataSize / (numChannels_ * bytesPerSample);

  // Read entire audio data into buffer
  std::vector<char> rawData(dataSize);
  file.read(rawData.data(), dataSize);
  if (file.gcount() != static_cast<std::streamsize>(dataSize)) {
    std::cerr << "Failed to read complete audio data" << std::endl;
    return false;
  }

  // Convert to float and store in audioBuffer_
  audioBuffer_.reserve(totalSamples_);

  for (size_t i = 0; i < totalSamples_; ++i) {
    float sample = 0.0f;

    if (bitsPerSample_ == 16) {
      int16_t rawSample = *reinterpret_cast<int16_t *>(
          rawData.data() + i * numChannels_ * bytesPerSample);
      sample = rawSample / 32768.0f;
    } else if (bitsPerSample_ == 32) {
      if (audioFormat == 3) {
        // IEEE float
        sample = *reinterpret_cast<float *>(rawData.data() +
                                            i * numChannels_ * bytesPerSample);
      } else {
        // 32-bit PCM
        int32_t rawSample = *reinterpret_cast<int32_t *>(
            rawData.data() + i * numChannels_ * bytesPerSample);
        sample = rawSample / 2147483648.0f;
      }
    } else if (bitsPerSample_ == 24) {
      const char *ptr = rawData.data() + i * numChannels_ * bytesPerSample;
      int32_t rawSample = (static_cast<int8_t>(ptr[2]) << 16) |
                          (static_cast<uint8_t>(ptr[1]) << 8) |
                          static_cast<uint8_t>(ptr[0]);
      sample = rawSample / 8388608.0f;
    } else if (bitsPerSample_ == 8) {
      uint8_t rawSample = *reinterpret_cast<uint8_t *>(
          rawData.data() + i * numChannels_ * bytesPerSample);
      sample = (rawSample - 128) / 128.0f;
    }

    audioBuffer_.push_back(sample);
  }

  return true;
}

bool WavFileSource::readBlock(Block &block) {
  block.setZero();

  if (audioBuffer_.empty() || currentSample_ >= audioBuffer_.size())
    return false;

  for (int i = 0; i < dsp::BLOCK_SIZE; ++i) {
    if (currentSample_ < audioBuffer_.size()) {
      block(i) = audioBuffer_[currentSample_++];
    } else {
      block(i) = 0.0f;
    }
  }

  return true;
}

void WavFileSource::writeWavHeader() {
  // Write placeholder header - will be updated in finalizeWavOutput
  char header[44] = {0};

  // RIFF chunk
  std::memcpy(header, "RIFF", 4);
  // Size will be filled later
  std::memcpy(header + 8, "WAVE", 4);

  // fmt chunk
  std::memcpy(header + 12, "fmt ", 4);
  uint32_t fmtSize = 16;
  std::memcpy(header + 16, &fmtSize, 4);
  uint16_t audioFormat = 3; // IEEE float
  std::memcpy(header + 20, &audioFormat, 2);
  uint16_t channels = 1;
  std::memcpy(header + 22, &channels, 2);
  uint32_t sampleRate = dsp::SAMPLE_RATE;
  std::memcpy(header + 24, &sampleRate, 4);
  uint32_t byteRate = sampleRate * channels * sizeof(float);
  std::memcpy(header + 28, &byteRate, 4);
  uint16_t blockAlign = channels * sizeof(float);
  std::memcpy(header + 32, &blockAlign, 2);
  uint16_t bitsPerSample = 32;
  std::memcpy(header + 34, &bitsPerSample, 2);

  // data chunk
  std::memcpy(header + 36, "data", 4);
  // Data size will be filled later

  outputFile_.write(header, 44);
}

void WavFileSource::writeBlock(const Block &block) {
  float maxVal = block.cwiseAbs().maxCoeff();
  if (samplesWritten_ == 0) {
    std::cout << "  First block max amplitude: " << maxVal << std::endl;
  }
  for (int i = 0; i < dsp::BLOCK_SIZE; ++i) {
    float sample = block(i);
    outputFile_.write(reinterpret_cast<const char *>(&sample), sizeof(float));
  }
  samplesWritten_ += dsp::BLOCK_SIZE;
}

void WavFileSource::finalizeWavOutput() {
  if (!outputFile_.is_open())
    return;

  uint32_t dataSize = samplesWritten_ * sizeof(float);
  uint32_t fileSize = dataSize + 36;

  // Update RIFF chunk size
  outputFile_.seekp(4);
  outputFile_.write(reinterpret_cast<const char *>(&fileSize), 4);

  // Update data chunk size
  outputFile_.seekp(40);
  outputFile_.write(reinterpret_cast<const char *>(&dataSize), 4);

  std::cout << "WAV output written: " << config_.outputPath << std::endl;
  std::cout << "  Samples: " << samplesWritten_ << std::endl;
  std::cout << "  Duration: "
            << (samplesWritten_ / static_cast<float>(dsp::SAMPLE_RATE))
            << " seconds" << std::endl;
}
