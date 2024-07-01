/**
 * @file MessageQueue.h
 * @author icbaek
 * @brief 
 * @version 0.1
 * @date 2023-08-07
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#ifndef SNOWDEER_MESSAGEQUEUE_H
#define SNOWDEER_MESSAGEQUEUE_H

#include <queue>
#include <mutex>
#include <condition_variable>

#include "eblog.h"

//#define DEBUG

using namespace std;

template<class T>
class MessageQueue {
 public:
  MessageQueue(void) : mQueue(), mMutex(), mCondition() {
    mIsLoop = true;
    eblog(LOG_LV,  "");
  }

  ~MessageQueue(void) {
    eblog(LOG_LV,  "");
  }

  void clear() {
    std::lock_guard<std::mutex> lock(mMutex);
    std::queue<T> emptyQueue;
    std::swap(mQueue, emptyQueue);
  }

  void enqueue(T t) {
    std::lock_guard<std::mutex> lock(mMutex);
    mQueue.emplace(t); //mQueue.push()
    mCondition.notify_one();
#ifdef DEBUG
    std::cout << "enqueue " << t << std::endl;
#endif
  }

  void destory() {
    mIsLoop = false;
    mCondition.notify_one();
  }

  T dequeue(void) {
    std::unique_lock<std::mutex> lock(mMutex);
    while((mIsLoop)&&(mQueue.empty())) {
      mCondition.wait(lock);
    }

    T val = std::move(mQueue.front()); //T val = mQueue.front();
    mQueue.pop();
#ifdef DEBUG
		std::cout << "dequeue " << val << std::endl;
#endif
    return val;
  }

 private:
  std::queue<T> mQueue;
  mutable std::mutex mMutex;
  std::condition_variable mCondition;
  bool mIsLoop;
};

#endif //SNOWDEER_MESSAGEQUEUE_H