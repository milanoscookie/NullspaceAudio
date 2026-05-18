#pragma once

#include <condition_variable>
#include <deque>

template <typename T> class BoundedQueue {
public:
  BoundedQueue() = default;
  ~BoundedQueue() = default;

  void push(const T &item) {}
  void pop(const T &item);

private:
  int maxCapacity_ = 0;
  std::condition_variable signalCv_;
  std::deque<T> items_;
};
