#pragma once

#include <queue>
#include <mutex>
#ifdef __linux__
#include <DeckLinkAPI.h>
#elif defined _WIN32
#include <windows.h>
#include <process.h>
#include <tchar.h>
#include <objbase.h>
#include <comutil.h>
#include <srvlib/io/video/sdi/DeckLinkAPI.h>
#endif
#include <cinder/gl/Texture.h>

namespace srvlib {

  namespace sdi {

    template<typename T>
    struct ThreadSafeQueue {

      ThreadSafeQueue(){ no_queue = nullptr; }

      void push(T v);
      T pop();
      size_t length();

      std::queue<T> queue;
      T no_queue;
      std::mutex m_lock;
    };

    template<typename T>
    void ThreadSafeQueue<T>::push(T v){

      if (!m_lock.try_lock()) return;
      no_queue = v;
      //queue.push(v);
      m_lock.unlock();

    }

    template<typename T>
    T ThreadSafeQueue<T>::pop() {

      T r = nullptr;

      if (!m_lock.try_lock()) return r;

      if (no_queue != nullptr){
        r = no_queue;
        no_queue = nullptr;
      }

      //if (!queue.empty()){
      //  r = queue.front();
      //  queue.pop();
      //}

      m_lock.unlock();

      return r;

    }

    template<typename T>
    size_t ThreadSafeQueue<T>::length() {
      while (!m_lock.try_lock()) {}
      size_t r = queue.size();
      m_lock.unlock();
      return r;
    }

    typedef ThreadSafeQueue<IDeckLinkVideoInputFrame *> FrameQueue;
  }
}
