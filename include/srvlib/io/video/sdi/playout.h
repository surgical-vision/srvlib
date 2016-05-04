#ifndef __PLAYOUT_H__
#define __PLAYOUT_H__

#ifdef __linux__
#include <pthread.h>
#include <DeckLinkAPI.h>
#elif defined _WIN32
#include <windows.h>
#include <process.h>
#include <tchar.h>
#include <objbase.h>
#include <comutil.h>
#include <srvlib/io/video/sdi/DeckLinkAPI.h>
#endif

#include <deque>

#include <srvlib/io/video/sdi/capture.h>
#include <srvlib/io/video/sdi/config.h>
#include <utility>
#include <mutex>

namespace srvlib {

  namespace sdi {

    class PlayoutDevice {

    public:

      //should remove width and height, get from display mode
      PlayoutDevice(const size_t device_index, BMDConfig &config, const BMDDisplayMode target_display_mode, const size_t frame_width, const size_t frame_height);
      ~PlayoutDevice();
      //void DisplayFrame(void *frameBytes);

      bool GetStreamFromDevice();

      void SetupDisplay(const size_t width, const size_t height);

      void SetDeckLinkOutput(IDeckLinkOutput *output);

      void DrawFrameToDisplay();

      IDeckLinkMutableVideoFrame *GetFrame() { return frame_; }

      void SetOffsetToStream(const size_t offset);

    protected:

      bool EnableVideoAndStartDisplay(BMDConfig *config, const BMDDisplayMode target_display_mode);

      IDeckLinkMutableVideoFrame *frame_;

      BMDDisplayModeSupport	display_mode_supported;

      BMDConfig config_;

      IDeckLinkOutput *output_;
      IDeckLinkConfiguration  *configuration_;

      const size_t device_index_;

      size_t frame_width_;
      size_t frame_height_;

    };

  }
}

#endif
