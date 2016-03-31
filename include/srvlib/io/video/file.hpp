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

#include <srvlib/io/video/video.hpp>

namespace srvlib {

  /**
  * @class VideoIO
  * @brief Simple video input output wrapper.
  * This class wraps up video input and output in a simple to use container. Interfaces to videos or image files.
  */
  class VideoFileIO : public VideoIO {

  public:

    /**
    * Open a input only version of the class - when we don't necessarily want to write anything.
    * @param[in] inpath The path to the input video file or image file.
    */
    explicit VideoFileIO(const std::string &inpath);
    
    /**
    * Open an input and output file.
    * @param[in] inpath The path to the input video file or image file.
    * @param[in] outpath The path to the output video file.
    */
    VideoFileIO(const std::string &inpath, const std::string &outpath);

    /**
    * Close the video streams.
    */
    virtual ~VideoFileIO();

    /**
    * Read the next frame. 
    * @return An empty matrix if not enabled or a black frame if there's nothing else to read.
    */
    virtual std::shared_ptr<VideoFrame> Read();

    /**
    * Read the next when we have a packed (side-by-side) stereo frame. Returns an empty matrix if not enabled or a black frame if there's nothing else to read.
    * @param[out] The left part of the frame we read.
    * @param[out] The right part of the frame we read.
    */
    virtual void Read(std::shared_ptr<VideoFrame>, std::shared_ptr<VideoFrame>);

    /**
    * Close the video streams.
    */
    virtual void CloseStreams();

  protected:

    cv::Mat image_input_; /**< If we read from an image file, it's stored here. */
    cv::VideoCapture cap_; /**< Video capture interface. */
    
  };


}