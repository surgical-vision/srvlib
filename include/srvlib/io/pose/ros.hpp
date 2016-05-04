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

#include <cinder/app/App.h>
#include <cinder/params/Params.h>

#include <fstream>
#include <vector>

#include <srvlib/utils/config_reader.hpp>
#include <srvlib/model/model.hpp>
#include <srvlib/view/camera.hpp>

#ifdef USE_DA_VINCI
#include <srvlib/model/davinci/davinci.hpp>
#endif

namespace srvlib {



}