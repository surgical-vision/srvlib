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

#include <srvlib/io/video/file.hpp>

using namespace srvlib;

VideoFileIO::VideoFileIO(const std::string &inpath){

   if (boost::filesystem::path(inpath).extension().string() == ".png" ||
    boost::filesystem::path(inpath).extension().string() == ".jpg" ||
    boost::filesystem::path(inpath).extension().string() == ".jpeg" ||
    boost::filesystem::path(inpath).extension().string() == ".bmp"){
    image_input_ = cv::imread(inpath);
    if (!image_input_.data) throw std::runtime_error("Error, could not open the image!");
    image_width_ = image_input_.cols;
    image_height_ = image_input_.rows;

  }
  else{
    cap_.open(inpath);
    if (!cap_.isOpened()) throw std::runtime_error("Error, could not open input video file");
    image_width_ = (size_t)cap_.get(CV_CAP_PROP_FRAME_WIDTH);
    image_height_ = (size_t)cap_.get(CV_CAP_PROP_FRAME_HEIGHT);
  }

  is_open_ = true;

}


VideoFileIO::VideoFileIO(const std::string &inpath, const std::string &outpath) : VideoFileIO(inpath) {

  VideoIO::Setup(image_width_, image_height_, outpath);
  
}

VideoFileIO::~VideoFileIO(){

  CloseStreams();

}

void VideoFileIO::CloseStreams(){

  if (cap_.isOpened())
    cap_.release();

  VideoIO::CloseStreams();

}

std::shared_ptr<VideoFrame> VideoFileIO::Read(){

  std::shared_ptr<VideoFrame>  f(new VideoFrame);
  if (!is_open_) {
    f->cpu_frame = cv::Mat::zeros(cv::Size(image_width_, image_height_), CV_8UC3);
    f->is_valid = false;
    return f;
  }

  

  if (cap_.isOpened()){
    cv::Mat m;
    cap_ >> m;
    if (m.data){
      f->cpu_frame = m;
      f->is_valid = true;
    }
    else{
      f->cpu_frame = cv::Mat::zeros(cv::Size(image_width_, image_height_), CV_8UC3);
      f->is_valid = false;
      is_open_ = false;
    }
  }
  else if (!image_input_.empty()){
    cv::Mat m = image_input_.clone();
    f->cpu_frame = m;
    f->is_valid = true;
  }

  return f;

}

void VideoFileIO::Read(std::shared_ptr<VideoFrame> left, std::shared_ptr<VideoFrame>  right){

  left.reset(new VideoFrame);
  right.reset(new VideoFrame);

  if (!is_open_) {
    left->cpu_frame = cv::Mat::zeros(cv::Size((int)image_width_ / 2, image_height_), CV_8UC3);
    left->is_valid = false;
    right->cpu_frame = cv::Mat::zeros(cv::Size((int)image_width_ / 2, image_height_), CV_8UC3);
    right->is_valid = false;
    return;
  }


  if (cap_.isOpened()){
    cv::Mat m;
    cap_ >> m;
    cv::Mat l = m(cv::Rect(0, 0, m.cols / 2, m.rows));
    cv::Mat r = m(cv::Rect(m.cols / 2, 0, m.cols / 2, m.rows));

    if (m.data){
      l.copyTo(left->cpu_frame);
      left->is_valid = true;
      r.copyTo(right->cpu_frame);
      right->is_valid = true;
    }
    else{
      left->cpu_frame = cv::Mat::zeros(cv::Size((int)image_width_ / 2, image_height_), CV_8UC3);
      right->cpu_frame = cv::Mat::zeros(cv::Size((int)image_width_ / 2, image_height_), CV_8UC3);
      left->is_valid = false;
      right->is_valid = false;
      is_open_ = false;
    }
  }
  
}