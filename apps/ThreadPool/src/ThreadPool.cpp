// Copyright @ 2016 Caoyang Jiang

#include <atomic>
#include <condition_variable>
#include <functional>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <vector>

/**
 * @brief      Thread pool. The processing model of thread pool is procedure +
 *             data.  To use the thread pool, the application must ensure the
 *             procedure only accepts a single parameter. The parameter is
 *             passed into procedure and then cast into appropriate type for
 *             further processing. Thread pool handles all task distribution and
 *             synchronization. The tasks are distributed in
 *             first-come-first-serve manner. To avoid waiting in the main
 *             thread, the AddTask()/AddTasks() do not use wait() function. The
 *             application is responsible for checking AddTask. Terminate() must
 *             be called to properly destroy the ThreadPool.
 * @ingroup    DashVideoDecoder
 */
class ThreadPool
{
 public:
  /**
   * @brief      Task data structure format for thread pool. The "arg" is used
   *             for passing input parameters and also for collecting output
   *             results.  ThreadPoolTask is an abstraction of procedure + data.
   * @ingroup    DashVideoDecoder.
   */
  class ThreadPoolTask
  {
   public:
    std::function<void(void*)> func;  // Thread callback function.
    void* arg;                        // Thread callback function argument.
  };

 public:
  // ThreadPool()
  // {
  // }
  // ~ThreadPool()
  // {
  // }

  /**
   * @brief      Intialize thread pool parameters. All parameters must > 0.
   *
   * @param[in]  threadcount  maximum number of threads in the pool.
   * @param[in]  maxtasksize  maximum number of tasks in the queue.
   */
  void Intialize(int threadcount, int maxtasksize)
  {
    threadcount_ = threadcount;
    maxtasksize_ = maxtasksize;
    taskscount_  = 0;
  }

  /**
   * @brief     Terminate thread pool.
   */
  void Terminate()
  {
    shutdown_ = true;
    consumercv_.notify_all();
    for (std::thread& thread : threads_) thread.join();
  }

  /**
   * @brief      Launch threads.
   */
  void CreateThreadPool()
  {
    shutdown_ = false;

    for (int i = 0; i < threadcount_; i++)
    {
      threads_.emplace_back(std::thread(&ThreadPool::RunThread, this));
    }
  }

  /**
   * @brief      Thread process. All threads executes the same code operating on
   *             different data (STMD). The task is described in the
   *             ThreadPoolTask structure.
   */
  void RunThread()
  {
    for (;;)
    {
      ThreadPoolTask task;

      {
        std::unique_lock<std::mutex> lock(tasksguard_);
        consumercv_.wait(lock, [this] { return !tasks_.empty() || shutdown_; });

        if (shutdown_) return;

        if (!tasks_.empty())
        {
          task = std::move(tasks_.front());
          tasks_.pop();
          taskscount_--;
        }
      }

      task.func(task.arg);
    }
  }

  /**
   * @brief      Adds a  single task to thread pool task queue.
   *
   * @param[in]  task  A thread task.
   *
   * @return    True if sucessfully adding task, false otherwise.
   */
  bool AddTask(const ThreadPoolTask& task)
  {
    std::cout << taskscount_ << " " << maxtasksize_ << std::endl;
    if (taskscount_ <= maxtasksize_)
    {
      std::unique_lock<std::mutex> lock(tasksguard_);
      tasks_.emplace(task);
      taskscount_++;
      std::cout << "add a task" << tasks_.size() << std::endl;
      lock.unlock();
      consumercv_.notify_all();
    }
    else
    {
      return false;
    }

    return true;
  }

  /**
   * @brief      Adds multiple tasks to thread pool task queue.
   *
   * @param[in]  tasks  Thread tasks.
   *
   * @return     True if task queue has enough space for all tasks.
   */
  bool AddTasks(const std::vector<ThreadPoolTask>& tasks)
  {
    if ((taskscount_ + static_cast<int>(tasks.size())) <= maxtasksize_)
    {
      std::unique_lock<std::mutex> lock(tasksguard_);

      for (size_t i = 0; i < tasks.size(); i++)
      {
        tasks_.emplace(tasks[i]);
      }
    }

    consumercv_.notify_all();
    return true;
  }

 private:
  std::vector<std::thread> threads_;
  std::queue<ThreadPoolTask> tasks_;
  std::mutex tasksguard_;
  std::condition_variable feedercv_;
  std::condition_variable consumercv_;
  std::atomic<int> taskscount_;
  bool shutdown_   = true;
  int maxtasksize_ = 0;
  int threadcount_ = 0;
};

struct DLTask
{
  // DLTask()
  // {
  // }
  // ~DLTask()
  // {
  // }
  // Input data
  std::string myid_;
  size_t input_;

  // Output data
  std::unique_ptr<uint8_t*> buffer_;
  size_t buffersize_ = 0;
};

class Downloader
{
 public:
  static void Download(void* data)
  {
    DLTask* task      = reinterpret_cast<DLTask*>(data);
    task->buffersize_ = task->input_;
    std::cout << task->input_ << std::endl;
    // sleep(2);
    std::cout << "done" << std::endl;
  }
};
int main()
{
  DLTask a, b;
  ThreadPool::ThreadPoolTask pta, ptb;

  a.myid_  = "Thread a";
  b.myid_  = "Thread b";
  a.input_ = 1;
  b.input_ = 2;

  pta.arg  = reinterpret_cast<void*>(&a);
  ptb.arg  = reinterpret_cast<void*>(&b);
  pta.func = Downloader::Download;
  ptb.func = Downloader::Download;

  ThreadPool tp;

  tp.Intialize(5, 2);
  tp.CreateThreadPool();
  tp.AddTask(pta);
  tp.AddTask(ptb);

  while (a.buffersize_ == 0 || b.buffersize_ == 0)
  {
  }
  tp.Terminate();

  std::cout << a.buffersize_ << " " << b.buffersize_ << std::endl;
  return 0;
}
