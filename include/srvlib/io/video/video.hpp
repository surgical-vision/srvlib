#pragma once

/**

srvlib - A robotics visualizer specialized for the da Vinci robotic system.
Copyright (C) 2014 Max Allan

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

**/

#include <opencv2/highgui/highgui.hpp>
#include <cinder/gl/Texture.h>

namespace srvlib {

  struct VideoFrame {

    size_t GetWidth() const {

      if (is_cpu_frame()){
        return cpu_frame.cols;
      }
      else if (is_gpu_frame()){
        return gpu_frame->getWidth() * 2; // GPU frame is half width!!!
      }
      else{
        return 0;
      }

    }

    size_t GetHeight() const {

      if (is_cpu_frame()){
        return cpu_frame.rows;
      }
      else if (is_gpu_frame()){
        return gpu_frame->getHeight();
      }
      else{
        return 0;
      }
    }

    bool is_cpu_frame() const {
      if (cpu_frame.data) return true;
      else return false;
    }

    bool is_gpu_frame() const {
      if (gpu_frame) return true;
      else return false;
    }

    cv::Mat cpu_frame;
    ci::gl::Texture2dRef gpu_frame;
    bool is_valid;

  };
  

  /**
  * @class VideoIO
  * @brief Simple video input output wrapper.
  * This class wraps up video input and output in a simple to use container. Interfaces to videos or image files.
  */
  class VideoIO {

  public:

    /**
    * Set up a default object which basically does nothing. Only useful for delayed opening.
    */
    VideoIO() : is_open_(false) {}
   
    /**
    * Open an output file.
    * @param[in] outpath The path to the output video file.
    */
    VideoIO(size_t image_width, size_t image_height, const std::string &outpath) { Setup(image_width, image_height, outpath); }

    void Setup(size_t image_width, size_t image_height, const std::string &outpath);

    /**
    * Close the video streams.
    */
    virtual ~VideoIO();

    /**
    * Read the next frame. 
    * @return An empty matrix if not enabled or a black frame if there's nothing else to read.
    */
    virtual std::shared_ptr<VideoFrame> Read() = 0;

    std::size_t GetWidth() { return image_width_; } 
    std::size_t GetHeight() { return image_height_; }


    /**
    * Read the next when we have a packed (side-by-side) stereo frame. Returns an empty matrix if not enabled or a black frame if there's nothing else to read.
    * @param[out] The left part of the frame we read.
    * @param[out] The right part of the frame we read.
    */
    virtual void Read(std::shared_ptr<VideoFrame> left, std::shared_ptr<VideoFrame> right) = 0;

    /**
    * Write the current frame.
    * @param[in] The current frame. Resizes it if's the wrong size.
    */
    void Write(const cv::Mat &frame);
    
    /**
    * Write the current stereo to a a packed (side-by-side) stereo frame. 
    * @param[in] The left part of the frame we want to write.
    * @param[in] The right part of the frame we want to write.
    */
    void Write(const cv::Mat &left_frame, const cv::Mat &right_frame);

    virtual void DrawOnWindow(std::shared_ptr<VideoFrame> frame, const ci::Rectf bounds, bool clear);

    /**
    * Close the video streams.
    */
    virtual void CloseStreams();

    /**
    * Check if this file is open.
    * @return True if it is, false otherwise.
    */
    bool IsOpen() const { return is_open_; }

  protected:

    cv::Mat image_input_; /**< If we read from an image file, it's stored here. */

    cv::VideoWriter writer_; /**< Video writer interface. */
    
    std::size_t image_width_; /**< The image width we are writing. */
    std::size_t image_height_; /**< The image height we are writing. */

    bool is_open_; /**< Boolean for whether we have opened the capture interface. */

  };


}