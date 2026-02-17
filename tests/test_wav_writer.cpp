// Tests for WavWriter
#include "test_harness.h"
#include "wav_writer.h"
#include <cstdio>
#include <cstring>
#include <fstream>

static const std::string TEST_WAV = "/tmp/test_wav_writer.wav";

TEST(open_creates_file) {
  std::remove(TEST_WAV.c_str());
  WavWriter w(TEST_WAV);
  ASSERT_TRUE(w.open());
  ASSERT_TRUE(w.isOpen());
  w.close();
  // File should exist
  std::ifstream f(TEST_WAV);
  ASSERT_TRUE(f.good());
}

TEST(samples_written_count) {
  std::remove(TEST_WAV.c_str());
  WavWriter w(TEST_WAV);
  w.open();
  ASSERT_EQ(w.getSamplesWritten(), 0u);

  Block b = Block::Ones();
  w.writeBlock(b);
  ASSERT_EQ(w.getSamplesWritten(), static_cast<size_t>(dsp::BLOCK_SIZE));

  w.writeBlock(b);
  ASSERT_EQ(w.getSamplesWritten(), static_cast<size_t>(2 * dsp::BLOCK_SIZE));
  w.close();
}

TEST(duration_correct) {
  std::remove(TEST_WAV.c_str());
  WavWriter w(TEST_WAV);
  w.open();
  // Write 1 second worth of blocks
  int blocksPerSecond = dsp::SAMPLE_RATE / dsp::BLOCK_SIZE;
  Block b = Block::Zero();
  for (int i = 0; i < blocksPerSecond; ++i)
    w.writeBlock(b);
  float dur = w.getDurationSeconds();
  ASSERT_NEAR(dur, 1.0f, 0.01f);
  w.close();
}

TEST(wav_header_valid) {
  std::remove(TEST_WAV.c_str());
  {
    WavWriter w(TEST_WAV);
    w.open();
    Block b = Block::Ones() * 0.5f;
    w.writeBlock(b);
    w.close();
  }
  // Read and verify RIFF header
  std::ifstream f(TEST_WAV, std::ios::binary);
  ASSERT_TRUE(f.good());
  char riff[4];
  f.read(riff, 4);
  ASSERT_TRUE(std::memcmp(riff, "RIFF", 4) == 0);

  // Skip 4 bytes (file size)
  f.seekg(8);
  char wave[4];
  f.read(wave, 4);
  ASSERT_TRUE(std::memcmp(wave, "WAVE", 4) == 0);
}

TEST(file_size_reasonable) {
  std::remove(TEST_WAV.c_str());
  {
    WavWriter w(TEST_WAV);
    w.open();
    Block b = Block::Zero();
    w.writeBlock(b);
    w.close();
  }
  std::ifstream f(TEST_WAV, std::ios::ate | std::ios::binary);
  auto size = f.tellg();
  // 44 bytes header + BLOCK_SIZE * 2 bytes (16-bit)
  size_t expected = 44 + dsp::BLOCK_SIZE * 2;
  ASSERT_NEAR(static_cast<float>(size), static_cast<float>(expected), 8.0f);
}

int main() {
  RUN_TEST(open_creates_file);
  RUN_TEST(samples_written_count);
  RUN_TEST(duration_correct);
  RUN_TEST(wav_header_valid);
  RUN_TEST(file_size_reasonable);
  PRINT_RESULTS();
  std::remove(TEST_WAV.c_str());
  return g_fails > 0 ? 1 : 0;
}
