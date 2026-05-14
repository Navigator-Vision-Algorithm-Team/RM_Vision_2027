#ifndef RM_OMNIPERCEPTION__THREAD_SAFE_QUEUE_HPP_
#define RM_OMNIPERCEPTION__THREAD_SAFE_QUEUE_HPP_

#include <condition_variable>
#include <mutex>
#include <queue>

namespace rm_omniperception
{
namespace tools
{

template <typename T>
class ThreadSafeQueue
{
public:
  explicit ThreadSafeQueue(size_t capacity = 10)
  : capacity_(capacity), stop_(false) {}

  bool push(T item)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    if (queue_.size() >= capacity_) {
      queue_.pop();  // drop oldest
    }
    queue_.push(std::move(item));
    lock.unlock();
    cond_.notify_one();
    return true;
  }

  bool pop(T & item)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    cond_.wait(lock, [this] { return !queue_.empty() || stop_; });
    if (stop_ && queue_.empty()) return false;
    item = std::move(queue_.front());
    queue_.pop();
    return true;
  }

  bool try_pop(T & item)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (queue_.empty()) return false;
    item = std::move(queue_.front());
    queue_.pop();
    return true;
  }

  bool empty() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return queue_.empty();
  }

  size_t size() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return queue_.size();
  }

  void stop()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stop_ = true;
    cond_.notify_all();
  }

  void drain(std::vector<T> & out)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    while (!queue_.empty()) {
      out.push_back(std::move(queue_.front()));
      queue_.pop();
    }
  }

private:
  std::queue<T> queue_;
  size_t capacity_;
  bool stop_;
  mutable std::mutex mutex_;
  std::condition_variable cond_;
};

}  // namespace tools
}  // namespace rm_omniperception

#endif  // RM_OMNIPERCEPTION__THREAD_SAFE_QUEUE_HPP_
