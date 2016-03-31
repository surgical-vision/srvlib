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

#include <srvlib/view/gui.hpp>
#include <cinder/app/App.h>

using namespace srvlib;

bool GUI::constructed_ = false;

std::unique_ptr<GUI> GUI::instance_;

void GUI::CreateDefaultGUI(const size_t &camera_image_width, const size_t &camera_image_height){

  ci::app::setFullScreen();

  subwindows_["UI"].reset(new SubWindow);
  subwindows_["Editor"].reset(new SubWindow);
  subwindows_["LeftEye"].reset(new SubWindow);
  subwindows_["RightEye"].reset(new SubWindow);
  subwindows_["3DViz"].reset(new SubWindow);
  subwindows_["MultiView"].reset(new SubWindow);
  
  subwindows_["UI"]->Init("UI", 0, 0, 0.2*ci::app::getWindowWidth(), 0.5*ci::app::getWindowHeight(), false);
  subwindows_["Editor"]->Init("Editor", 0, 0.5*ci::app::getWindowHeight(), 0.2*ci::app::getWindowWidth(), 0.5*ci::app::getWindowHeight(), false);
  subwindows_["LeftEye"]->Init("Left Eye", subwindows_["UI"]->GetRect().x2, 0, camera_image_width, camera_image_height, 720, 576, true);
  subwindows_["RightEye"]->Init("Right Eye", subwindows_["UI"]->GetRect().x2, camera_image_height, camera_image_width, camera_image_height, 720, 576, true);
  subwindows_["3DViz"]->Init("3D Viz", subwindows_["LeftEye"]->GetRect().x2, 0, 576, 576, true);
  subwindows_["MultiView"]->Init("MultiView", subwindows_["LeftEye"]->GetRect().x2, 576, 576, 576, true);
 
 

}

std::shared_ptr<SubWindow> GUI::GetWindowByName(const std::string &name){

  return subwindows_[name];

}

void GUI::DrawUI(){

  subwindows_["UI"]->Draw(params_);

}

ci::params::InterfaceGlRef GUI::GetUIParams(){

  if (!params_) params_ = ci::params::InterfaceGl::create(ci::app::getWindow(), "DVRK Visualizer", ci::app::toPixels(glm::ivec2(200, 300)));
  return params_;

}

//void GUI::AddPoseGrabberButton(const size_t item_idx, bool is_camera){
//
//  if (is_camera){
//    gui_->addButton("Edit camera pose", std::bind(&vizApp::editPoseButton, this, 0));
//  }
//
//  for (size_t i = 0; i < trackables_.size(); ++i){
//    std::stringstream ss;
//    ss << "Edit instrument " << i << " pose";
//    gui_->addButton(ss.str(), std::bind(&vizApp::editPoseButton, this, i + 1));
//  }
//
//
//}


GUI::GUI(){}

GUI::~GUI(){}

void GUI::Destroy(){

  instance_.reset();
  constructed_ = false;

}

GUI::GUI(const GUI &that){}

GUI &GUI::operator=(const GUI &that){

  // check for self-assignment
  if (this == &that) return *this;

  //if we aren't self-assigning then something is wrong.
  throw(std::runtime_error("Error, attempting to construct a new TTrack.\n"));
  return *this;

}

GUI &GUI::Instance(){
  if (!constructed_){
    instance_.reset(new GUI());
    constructed_ = true;
  }
  return *(instance_.get());
}

