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
** IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
** FITNESS FOR A PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT
** SHALL THE COPYRIGHT HOLDERS OR ANYONE DISTRIBUTING THE SOFTWARE BE LIABLE
** FOR ANY DAMAGES OR OTHER LIABILITY, WHETHER IN CONTRACT, TORT OR OTHERWISE,
** ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
** DEALINGS IN THE SOFTWARE.
** -LICENSE-END-
*/

#ifndef __CAPTURE_H__
#define __CAPTURE_H__

#ifdef __linux__
#include <pthread.h>
#include <DeckLinkAPI.h>
#elif defined _WIN32
#include <windows.h>
#include <process.h>
#include <tchar.h>
#include <objbase.h>
#include <comutil.h>
#include <srvlib/io/video/sdi/DeckLinkAPI_h.h>
#endif


#include <srvlib/io/video/sdi/delegate.h>
#include <srvlib/io/video/sdi/config.h>
#include <srvlib/io/video/sdi/frame_queue.h>

namespace srvlib {

  namespace sdi {

    class CaptureDevice {

    public:

      CaptureDevice(int index, std::shared_ptr<FrameQueue> frame_queue, BMDConfig &config);
      ~CaptureDevice();

      void Destroy();

      void SetupDelegate(std::shared_ptr<FrameQueue> frame_queue);

      bool EnableVideoAndStartStream(BMDConfig *config);

      bool StartStream();

      void StopStream();

      size_t GetWidth() const;
      size_t GetHeight() const;



    protected:

      /**
       * C
       *
       */

      bool stream_running_;

      bool GetStreamFromDevice(BMDConfig *config);

      bool GetDisplayModeFromDevice(IDeckLink *device, BMDConfig *config);

      DeckLinkCaptureDelegate *delegate_;

      int device_index_;
      BMDConfig config_;

      IDeckLinkDisplayMode *display_mode_;
      IDeckLinkInput *input_;

    };

  }
}



#endif