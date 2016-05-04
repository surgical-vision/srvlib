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

      ThreadSafeQueue(){ }

      void push(T v);
      T pop();

      std::queue<T> queue;
      std::mutex m_lock;
    };

    template<typename T>
    void ThreadSafeQueue<T>::push(T v){

      if (!m_lock.try_lock()) return;
      queue.push(v);
      m_lock.unlock();

    }

    template<typename T>
    T ThreadSafeQueue<T>::pop() {

      T r = nullptr;

      if (!m_lock.try_lock()) return r;

      if (!queue.empty()){
        r = queue.front();
        queue.pop();
      }

      m_lock.unlock();

      return r;


    }

    typedef ThreadSafeQueue<IDeckLinkVideoInputFrame *> FrameQueue;
  }
}
