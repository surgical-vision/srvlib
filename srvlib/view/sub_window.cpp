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

#include <srvlib/view/sub_window.hpp>
#include <cinder/app/App.h>
#include <cinder/CinderOpenCV.h>

using namespace srvlib;

std::string SubWindow::output_directory;

SubWindow *SubWindow::Init(const std::string &name, int start_x, int start_y, int eye_width, int eye_height, bool can_save){

  return Init(name, start_x, start_y, eye_width, eye_height, eye_width, eye_height, can_save);

}

SubWindow *SubWindow::Init(const std::string &name, int start_x, int start_y, int eye_width, int eye_height, int draw_width, int draw_height, bool can_save){

  name_ = name; 
  can_save_ = can_save;

  frame_count_ = 0;
  file_count_ = 0;

  window_coords_ = ci::Rectf((float)start_x, (float)start_y, (float)start_x + (float)draw_width, (float)start_y + (float)draw_height);
  framebuffer_ = ci::gl::Fbo::create(eye_width, eye_height);
  //framebuffer_->getColorTexture()->setTopDown(true);

  static size_t n = 0;
  std::stringstream ss;
  ss << "Save Window " << n;
  n++;

  if (can_save_){
    save_params_ = ci::params::InterfaceGl::create(ci::app::getWindow(), ss.str(), ci::app::toPixels(glm::ivec2(100, 100)));
    save_params_->addButton("Save contents to file", std::bind(&SubWindow::InitSavingWindow, this, 0));
    save_params_->show();
  }

  //srvlibApp::AddSubWindow(this);
  return this;

}

void SubWindow::Bind() {

  framebuffer_->bindFramebuffer();

}

//void SubWindow::ReplaceFrame(const cv::Mat &m){
//
//  cv::Mat mm = m;
//  framebuffer_->getTexture().update((ci::Surface)fromOcv(mm));
//
//}

void SubWindow::BindAndClear(){

  Bind();
  ci::gl::clear(ci::Color(0, 0, 0));

}

void SubWindow::UnBind() {
  framebuffer_->unbindFramebuffer();
}

bool SubWindow::CanSave() const {

  return can_save_; 

}


bool SubWindow::IsSaving() const {
  
  return writer_.isOpened();

}

void SubWindow::WriteFrameToFile(){

  cv::Mat window;// = toOcv(framebuffer_->getTexture());
  
  cv::Mat flipped_window;
  cv::flip(window, flipped_window, 0);

  if (frame_count_ == 2000){
    file_count_++;
    frame_count_ = 0;
    CloseStream();
    InitSavingWindow(file_count_);
  }
    
  frame_count_++;
  writer_.write(window);

}

void SubWindow::Draw(ci::params::InterfaceGlRef params){

  Draw(params, GetRectWithBuffer().getUpperLeft(), glm::ivec2(Width(), Height()));

}

void SubWindow::Draw(ci::params::InterfaceGlRef params, const glm::ivec2 tl, const glm::ivec2 size){

  if (!params->isVisible()) return;

  std::stringstream position;
  position << "position='" << tl[0] << " " << tl[1] << "'";
  params->setOptions("", position.str());
  std::stringstream ssize;
  ssize << "size='" << size[0] << " " << size[1] << "'";
  params->setOptions("", ssize.str());
  params->draw();

}

void SubWindow::Draw(){
  
  ci::Rectf window_with_buffer = GetRectWithBuffer();

  cv::Mat m = toOcv(framebuffer_->getColorTexture()->createSource());

  ci::gl::draw(framebuffer_->getColorTexture(), window_with_buffer);

  if (can_save_){
    Draw(save_params_, window_with_buffer.getUpperLeft() + glm::vec2(10, 10), glm::ivec2(200, 50));
  }

}


void SubWindow::CloseStream(){

  if (writer_.isOpened())
    writer_.release();
  
}

void SubWindow::InitSavingWindow(const size_t vid_file_idx){

  std::string &save_dir = output_directory;
  if (!boost::filesystem::exists(save_dir)) {
    boost::filesystem::create_directory(save_dir);
  }

  std::string name = name_;
  std::replace(name.begin(), name.end(), ' ', '_');
  std::stringstream filepath;
  filepath << save_dir + "/" + name + "_" << vid_file_idx << ".avi";

  writer_.open(filepath.str(), CV_FOURCC('D', 'I', 'B', ' '), 25, cv::Size(framebuffer_->getWidth(), framebuffer_->getHeight()));

}