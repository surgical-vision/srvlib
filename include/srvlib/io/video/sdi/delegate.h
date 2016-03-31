/* -LICENSE-START-
** Copyright (c) 2013 Blackmagic Design
**
** Permission is hereby granted, free of charge, to any person or organization
** obtaining a copy of the software and accompanying documentation covered by
** this license (the "Software") to use, reproduce, display, distribute,
** execute, and transmit the Software, and to prepare derivative works of the
** Software, and to permit third-parties to whom the Software is furnished to
** do so, all subject to the following:
**
** The copyright notices in the Software and this entire statement, including
** the above license grant, this restriction and the following disclaimer,
** must be included in all copies of the Software, in whole or in part, and
** all derivative works of the Software, unless such copies or derivative
** works are solely in the form of machine-executable object code generated by
** a source language processor.
**
** THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
** IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,`
** FITNESS FOR A PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT
** SHALL THE COPYRIGHT HOLDERS OR ANYONE DISTRIBUTING THE SOFTWARE BE LIABLE
** FOR ANY DAMAGES OR OTHER LIABILITY, WHETHER IN CONTRACT, TORT OR OTHERWISE,
** ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
** DEALINGS IN THE SOFTWARE.
** -LICENSE-END-
*/

#ifndef __DELEGATE_H__
#define __DELEGATE_H__

#ifdef __linux__
#include <pthread.h>
#include <DeckLinkAPI.h>
#include <sys/time.h>
#elif defined _WIN32
#include <windows.h>
#include <process.h>
#include <tchar.h>
#include <objbase.h>
#include <comutil.h>
#include <srvlib/io/video/sdi/DeckLinkAPI_h.h>
#endif
#include <map>
#include <vector>

#include <cinder/gl/Context.h>

#define GL_GLEXT_PROTOTYPES
#include <GL/gl.h>
#ifdef __linux__
#include <GL/glut.h>
#endif
#include <GL/glu.h>

#include <atomic>
#include <functional>

#include <srvlib/io/video/sdi/frame_queue.h>

namespace srvlib {

  namespace sdi {

    class DeckLinkCaptureDelegate : public IDeckLinkInputCallback {

    public:

      DeckLinkCaptureDelegate(std::shared_ptr<FrameQueue> frame_queue);
      ~DeckLinkCaptureDelegate();

      virtual HRESULT STDMETHODCALLTYPE QueryInterface(REFIID iid, LPVOID *ppv) { return E_NOINTERFACE; }
      virtual ULONG STDMETHODCALLTYPE AddRef(void);
      virtual ULONG STDMETHODCALLTYPE  Release(void);
      virtual HRESULT STDMETHODCALLTYPE VideoInputFormatChanged(BMDVideoInputFormatChangedEvents, IDeckLinkDisplayMode*, BMDDetectedVideoInputFormatFlags);
      virtual HRESULT STDMETHODCALLTYPE VideoInputFrameArrived(IDeckLinkVideoInputFrame*, IDeckLinkAudioInputPacket*);

      void SetDeckLinkInput(IDeckLinkInput *deck_link_input) { deck_link_input_ = deck_link_input; }

    private:

      std::shared_ptr<FrameQueue> frame_queue_;

      IDeckLinkInput *deck_link_input_; //DOES NOT OWN!

      ULONG ref_count_;

#ifdef __linux__
      pthread_mutex_t mutex_;
#else
      std::mutex mutex_;
#endif

    };

  }

}


#endif