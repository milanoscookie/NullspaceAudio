#include <chrono>
#include <condition_variable>
#include <ctime>
#include <deque>
#include <mutex>
#include <thread>

template <typename T> class BoundedQueue {
public:
  BoundedQueue() {}
  ~BoundedQueue() {}

  void push(const T &item) {}
  void pop(const T &item);

private:
  int max_capacity_;
  std::condition_variable signal_cv;
  std::deque<T> items_;
};
