#pragma once
#include <atomic>
#include <cstdint>

template <typename T> class DoubleBufferSPSC {
public:
  DoubleBufferSPSC() = default;

  void publish(const T &value) {
    const uint64_t next = seq_.load(std::memory_order_relaxed) + 1;
    const int idx = indexFromSeq_(next);

    buf_[idx] = value; // copy assignment? may be an issue; For templated
                       // Eigen::Maxtrix, this is not an issue

    seq_.store(next, std::memory_order_release);
  }

  T &beginWrite() {
    pending_seq_ = seq_.load(std::memory_order_relaxed) + 1;
    pending_idx_ = indexFromSeq_(pending_seq_);
    return buf_[pending_idx_];
  }

  void commit() {
    // beginWrite was called?
    seq_.store(pending_seq_, std::memory_order_release);
  }

  bool tryRead(T &out) const {
    const uint64_t s = seq_.load(std::memory_order_acquire);
    if (s == last_read_seq_)
      return false;

    out = buf_[indexFromSeq_(s)];
    last_read_seq_ = s;
    return true;
  }

  void readLatest(T &out) const {
    const uint64_t s = seq_.load(std::memory_order_acquire);
    out = buf_[indexFromSeq_(s)];
    last_read_seq_ = s;
  }

  bool hasNew() const {
    const uint64_t s = seq_.load(std::memory_order_acquire);
    return s != last_read_seq_;
  }

  uint64_t sequence() const { return seq_.load(std::memory_order_acquire); }

private:
  static int indexFromSeq_(uint64_t seq) {
    return static_cast<int>(seq & 1ull);
  }

  // CPU cache
  alignas(64) mutable T buf_[2] = {T{}, T{}};
  alignas(64) std::atomic<uint64_t> seq_{0};

  mutable uint64_t last_read_seq_ = 0;

  uint64_t pending_seq_ = 0;
  int pending_idx_ = 0;
};
