#include "wav_writer.h"
#include <cstring>
#include <iostream>

WavWriter::WavWriter(const std::string &path, const Config &config)
    : path_(path), config_(config) {}

WavWriter::~WavWriter() {
  if (file_.is_open()) {
    close();
  }
}

bool WavWriter::open() {
  file_.open(path_, std::ios::binary);
  if (!file_.is_open()) {
    std::cerr << "WavWriter: Failed to open file: " << path_ << std::endl;
    return false;
  }

  writeHeader();
  return true;
}

void WavWriter::writeHeader() {
  // Write WAV header with placeholder sizes
  char header[44];
  std::memset(header, 0, 44);

  // RIFF chunk descriptor
  std::memcpy(header, "RIFF", 4);
  // Chunk size (placeholder - will be updated in finalize)
  uint32_t chunkSize = 0;
  std::memcpy(header + 4, &chunkSize, 4);
  std::memcpy(header + 8, "WAVE", 4);

  // fmt sub-chunk
  std::memcpy(header + 12, "fmt ", 4);
  uint32_t subchunk1Size = 16; // PCM
  std::memcpy(header + 16, &subchunk1Size, 4);

  uint16_t audioFormat =
      (config_.bitsPerSample == 32) ? 3 : 1; // 3 = IEEE float, 1 = PCM
  std::memcpy(header + 20, &audioFormat, 2);

  uint16_t numChannels = static_cast<uint16_t>(config_.numChannels);
  std::memcpy(header + 22, &numChannels, 2);

  uint32_t sampleRate = static_cast<uint32_t>(config_.sampleRate);
  std::memcpy(header + 24, &sampleRate, 4);

  uint32_t byteRate = sampleRate * numChannels * config_.bitsPerSample / 8;
  std::memcpy(header + 28, &byteRate, 4);

  uint16_t blockAlign = numChannels * config_.bitsPerSample / 8;
  std::memcpy(header + 32, &blockAlign, 2);

  uint16_t bitsPerSample = static_cast<uint16_t>(config_.bitsPerSample);
  std::memcpy(header + 34, &bitsPerSample, 2);

  // data sub-chunk
  std::memcpy(header + 36, "data", 4);
  dataChunkPos_ = 40;    // Position of data size field
  uint32_t dataSize = 0; // Placeholder
  std::memcpy(header + 40, &dataSize, 4);

  file_.write(header, 44);
}

void WavWriter::writeBlock(const Block &block) {
  writeSamples(block.data(), dsp::BLOCK_SIZE);
}

void WavWriter::writeSamples(const float *samples, size_t count) {
  if (!file_.is_open())
    return;

  if (config_.bitsPerSample == 32) {
    // Write as 32-bit float
    file_.write(reinterpret_cast<const char *>(samples), count * sizeof(float));
  } else if (config_.bitsPerSample == 16) {
    // Convert to 16-bit PCM
    for (size_t i = 0; i < count; ++i) {
      float sample = samples[i];
      // Clamp to [-1, 1]
      if (sample > 1.0f)
        sample = 1.0f;
      if (sample < -1.0f)
        sample = -1.0f;

      int16_t pcmSample = static_cast<int16_t>(sample * 32767.0f);
      file_.write(reinterpret_cast<const char *>(&pcmSample), sizeof(int16_t));
    }
  }

  samplesWritten_ += count;
}

void WavWriter::writeSamples(const std::vector<float> &samples) {
  writeSamples(samples.data(), samples.size());
}

void WavWriter::close() {
  if (!file_.is_open())
    return;

  finalizeHeader();
  file_.close();

  std::cout << "WavWriter: Saved " << path_ << std::endl;
  std::cout << "  Samples: " << samplesWritten_ << std::endl;
  std::cout << "  Duration: " << getDurationSeconds() << " seconds"
            << std::endl;
}

void WavWriter::finalizeHeader() {
  if (!file_.is_open())
    return;

  size_t bytesPerSample = config_.bitsPerSample / 8;
  uint32_t dataSize = static_cast<uint32_t>(
      samplesWritten_ * config_.numChannels * bytesPerSample);
  uint32_t chunkSize = dataSize + 36; // 36 = header size - 8

  // Update RIFF chunk size
  file_.seekp(4);
  file_.write(reinterpret_cast<const char *>(&chunkSize), 4);

  // Update data chunk size
  file_.seekp(dataChunkPos_);
  file_.write(reinterpret_cast<const char *>(&dataSize), 4);

  // Seek to end
  file_.seekp(0, std::ios::end);
}
