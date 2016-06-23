#pragma once

#include <cinder/gl/Fbo.h>

#include <srvlib/io/video/video.hpp>
#include "capture.h"
#include "frame_queue.h"
#include "playout.h"

namespace srvlib {

  namespace sdi {

    class SDICameraIO : public VideoIO {

    public:

      /**
      * Open a input only version of the class - when we don't necessarily want to write anything.
      * @param[in] inpath The path to the input video file or image file.
      */
      SDICameraIO();

      virtual ~SDICameraIO();

      void SetupInput(const std::string &input_file);
      void SetupInput(const size_t device_index, BMDConfig &config, const size_t width, const size_t height);
      void SetupOutput(const size_t device_index, BMDConfig &config, const BMDDisplayMode target_display_mode, const size_t frame_width, const size_t frame_height, const size_t offset);
      void SetOffsetToPlayout(const size_t offset_val);

      /**
      * Read the next frame.
      * @return An empty matrix if not enabled or a black frame if there's nothing else to read.
      */
      virtual std::shared_ptr<VideoFrame> Read() override;

      void Display(std::shared_ptr<VideoFrame> frame);

      void Display();

      /**
      * Read the next when we have a packed (side-by-side) stereo frame. Returns an empty matrix if not enabled or a black frame if there's nothing else to read.
      * @param[out] The left part of the frame we read.
      * @param[out] The right part of the frame we read.
      */
      virtual void Read(std::shared_ptr<VideoFrame> left, std::shared_ptr<VideoFrame> right) override;

      virtual void DrawOnWindow(std::shared_ptr<VideoFrame> frame, const ci::Rectf bounds, bool clear) override;
      
    protected:


      size_t camera_id_;
      size_t display_id_;
      std::shared_ptr<CaptureDevice> capture_;
      std::shared_ptr<PlayoutDevice> playout_;
      std::shared_ptr<FrameQueue> frame_queue_;

      ci::gl::FboRef fbo_;
      ci::gl::GlslProgRef ycrcb_shader_;
      //ci::gl::Texture2dRef texture_;

    };

  }
}