#ifndef PORT_COUNTING_SEM_HPP
#define PORT_COUNTING_SEM_HPP

#include <condition_variable>
#include <deque>
#include <mutex>
#include <thread>

// Counting semaphore using std::condition_variable and std::mutex
class CountingSemaphore {
public:
  CountingSemaphore(int count = 0,
                    int max_count = std::numeric_limits<int>::max())
      : count(count), max_count(max_count) {}

  ~CountingSemaphore() {}

  void notify() {
    std::unique_lock<std::mutex> lock(mutex, std::adopt_lock);
    if (count < max_count) {
      count++;
      condition.notify_one();
    }
  }

  void wait() {
    std::unique_lock<std::mutex> lock(mutex, std::adopt_lock);
    while (count == 0) { // Handle spurious wake-ups.
      condition.wait(lock);
    }
    count--;
  }

  bool try_wait() {
    std::unique_lock<std::mutex> lock(mutex, std::adopt_lock);
    if (count > 0) {
      count--;
      return true;
    }
    return false;
  }

  void lock_mutex() { mutex.lock(); }

  void unlock_mutex() { mutex.unlock(); }

  void reset(int count = 0) {
    std::unique_lock<std::mutex> lock(mutex);
    this->count = count;
  }

private:
  std::mutex mutex;
  std::condition_variable condition;
  int count;
  int max_count;
};

class Mailbox {
public:
  Mailbox(int initial_count = 0,
          int max_count = std::numeric_limits<int>::max())
      : count(0), max_count(max_count) {
    const int loops = std::min(initial_count, max_count);
    while (count != loops) {
      send();
    }
  }

  ~Mailbox() {
    std::unique_lock<std::mutex> lock(mutex);
    while (!queue.empty()) {
      void *pMsg = queue.front();
      queue.pop_front();
      count--;
      delete pMsg;
    }
  }

  bool send(void *value = nullptr) {
    bool ret = false;
    std::unique_lock<std::mutex> lock(mutex);
    if (count < max_count) {
      count++;
      queue.push_back(value);
      condition.notify_one();
      ret = true;
    }
    return ret;
  }

  bool receive(void **ppMsg = nullptr) {
    void *pMsg = nullptr;
    std::unique_lock<std::mutex> lock(mutex);
    while (queue.empty()) {
      condition.wait(lock);
    }
    pMsg = queue.front();
    queue.pop_front();
    count--;

    if (ppMsg) {
      *ppMsg = pMsg;
    }
    return true;
  }

  bool try_receive(void **ppMsg = nullptr) {
    void *pMsg = nullptr;
    std::unique_lock<std::mutex> lock(mutex);
    if (queue.empty()) {
      return false;
    }
    pMsg = queue.front();
    queue.pop_front();
    count--;

    if (ppMsg) {
      *ppMsg = pMsg;
    }
    return true;
  }

private:
  std::mutex mutex;
  std::condition_variable condition;
  std::deque<void *> queue;

  int count;
  int max_count;
};

#endif