#pragma once

#include <array>
#include <cstddef>

template <typename T, size_t Capacity> class RingBuffer {
public:
  void push_back(const T &v) {
    buffer_[tail_] = v;
    tail_ = (tail_ + 1) % Capacity;

    if (size_ < Capacity) {
      ++size_;
    } else {
      head_ = (head_ + 1) % Capacity; // overwrite oldest
    }
  }

  void pop_front() {
    if (size_ == 0)
      return;
    head_ = (head_ + 1) % Capacity;
    --size_;
  }

  const T *front() const {
    if (size_ == 0)
      return nullptr;
    return &buffer_[head_];
  }

  const T *back() const {
    if (size_ == 0)
      return nullptr;
    return &buffer_[(tail_ + Capacity - 1) % Capacity];
  }

  const T *from_back(size_t k) const {
    if (k >= size_)
      return nullptr;
    return &buffer_[(tail_ + Capacity - 1 - k) % Capacity];
  }

  const T *from_front(size_t k) const {
    if (k >= size_)
      return nullptr;
    return &buffer_[(head_ + k) % Capacity];
  }

  // k = 0 -> front (oldest), k = size-1 -> newest
  const T &operator[](size_t k) const {
    return buffer_[(head_ + k) % Capacity];
  }

  size_t size() const { return size_; }
  constexpr size_t capacity() const { return Capacity; }

  void clear() { head_ = tail_ = size_ = 0; }

private:
  std::array<T, Capacity> buffer_;
  size_t head_ = 0;
  size_t tail_ = 0;
  size_t size_ = 0;
};
