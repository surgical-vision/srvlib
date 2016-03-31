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

#include <opencv2/highgui/highgui_c.h>
#include <opencv2/opencv.hpp>
#include <boost/filesystem.hpp>

#include <srvlib/io/video/video.hpp>
#include <srvlib/view/renderer.hpp>
#include <cinder/gl/gl.h>
#include <cinder/CinderOpenCV.h>

using namespace srvlib;

void VideoIO::Setup(size_t image_width, size_t image_height, const std::string &outpath) {
  
  image_width_ = image_width;
  image_height_ = image_height;

  writer_.open(outpath, CV_FOURCC('M', 'J', 'P', 'G'), 25, cv::Size((int)image_width_, (int)image_height_));

  if (!writer_.isOpened()) throw std::runtime_error("Error, could not open output video file");

  is_open_ = true;

}

VideoIO::~VideoIO(){

  CloseStreams();

}

void VideoIO::CloseStreams(){

  if (writer_.isOpened())
    writer_.release();

}


void VideoIO::Write(const cv::Mat &frame){

  if (frame.size() != cv::Size((int)image_width_, (int)image_height_)){
    cv::Mat resized_frame;
    cv::resize(frame, resized_frame, cv::Size((int)image_width_, (int)image_height_));
    writer_ << resized_frame;
  }
  else{
    writer_ << frame;
  }


}


void VideoIO::Write(const cv::Mat &left_frame, const cv::Mat &right_frame){

  cv::Mat frame((int)image_height_, (int)image_width_, CV_8UC3);
  cv::Mat lf = frame(cv::Rect(0, 0, left_frame.cols, left_frame.rows));
  cv::Mat rf = frame(cv::Rect(left_frame.cols, 0, left_frame.cols, frame.rows));
  left_frame.copyTo(lf);
  right_frame.copyTo(rf);
  writer_ << frame;

}


void VideoIO::DrawOnWindow(std::shared_ptr<VideoFrame> frame, const ci::Rectf bounds, bool clear){

  if (clear)
    ci::gl::clear(ci::Color(0.5, 0.1, 0.0), true);
  
  if (frame->is_cpu_frame())
    renderer::DrawTexture(ci::gl::Texture2d::create(cinder::fromOcv(frame->cpu_frame)), glm::ivec2(bounds.getWidth(), bounds.getHeight()));
  else if (frame->is_gpu_frame())
    renderer::DrawTexture(frame->gpu_frame, glm::ivec2(bounds.getWidth(), bounds.getHeight()));

}