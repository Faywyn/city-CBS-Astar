#pragma once

#include <functional>
#include <future>
#include <queue>
#include <thread>

class ThreadPool {
public:
  ThreadPool(size_t numThreads);
  ~ThreadPool();

  size_t workersCount() { return workers.size(); }

  template <class F, class... Args>
  auto enqueue(F &&f, Args &&...args) -> std::future<std::invoke_result_t<F, Args...>> {
    using return_type = std::invoke_result_t<F, Args...>;

    auto taskPtr =
        std::make_shared<std::packaged_task<return_type()>>(std::bind(std::forward<F>(f), std::forward<Args>(args)...));

    std::future<return_type> res = taskPtr->get_future();
    {
      std::unique_lock<std::mutex> lock(queueMutex);
      if (stop)
        throw std::runtime_error("enqueue sur un ThreadPool déjà stoppé !");
      tasks.emplace([taskPtr]() { (*taskPtr)(); });
    }
    condition.notify_one();
    return res;
  }

private:
  std::vector<std::thread> workers;
  std::queue<std::function<void()>> tasks;

  std::mutex queueMutex;
  std::condition_variable condition;
  bool stop;
};
