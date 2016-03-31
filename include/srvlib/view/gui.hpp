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

#include <cinder/gl/gl.h>
#include <cinder/gl/Fbo.h>
#include <cinder/gl/Texture.h>
#include <cinder/gl/GlslProg.h>
#include <cinder/params/Params.h>

#include <opencv2/highgui/highgui.hpp>
#include "sub_window.hpp"

namespace srvlib {

  /**
  * @class SubWindow
  * @brief Sub window regions to draw to in the main viewer.
  * Allows the main window to be split up into different viewports.
  */
  class GUI {


  public:

    GUI();
    ~GUI();
    GUI &operator=(const GUI &);
    GUI(const GUI &);
    static GUI &Instance(); 
    void Destroy();
    void CreateDefaultGUI(const size_t &camera_image_width, const size_t &camera_image_height);

    void DrawUI();

    ci::params::InterfaceGlRef GetUIParams();

    std::shared_ptr<SubWindow> GetWindowByName(const std::string &name);

    //void AddPoseGrabberButton(const size_t item_idx, bool is_camera);

  protected:

    std::map< std::string, std::shared_ptr<SubWindow> > subwindows_;

    static bool constructed_;
    static std::unique_ptr<GUI> instance_;

    ci::params::InterfaceGlRef params_;

  };




}