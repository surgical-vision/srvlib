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

#include <boost/filesystem.hpp>
#include <cinder/ImageIo.h>
#include <cinder/app/App.h>
#include "cinder/app/RendererGl.h"
#include "cinder/gl/gl.h"

#include <srvlib/model/model.hpp>

using namespace srvlib;

ci::JsonTree Model::OpenFile(const std::string &datafile_path) const {

  boost::filesystem::path p(datafile_path);

  if (p.extension().string() == ".json"){

    ci::JsonTree loader(ci::loadFile(datafile_path));

    return loader;

  }
  else{

    throw std::runtime_error("Error, unsupported file type");
  
  }

}

Pose Model::GetPose() const {

  return base_pose_;

}

void Model::InternalDraw(Node::ConstPtr rd, const float inc) const {

  ci::gl::pushModelView();
  
  ci::gl::multModelMatrix(rd->GetWorldTransform(base_pose_));

  ci::gl::ScopedTextureBind tex_scope(rd->GetTexture());
  
  ci::gl::draw(rd->GetMesh());

  ci::gl::popModelView();

}

void Model::LoadComponent(const ci::JsonTree &tree, Node::Ptr target, const std::string &root_dir){

  boost::filesystem::path obj_file = boost::filesystem::path(root_dir) / boost::filesystem::path(tree["obj-file"].getValue<std::string>());
  if (!boost::filesystem::exists(obj_file)) throw(std::runtime_error("Error, the file doesn't exist!\n"));

  boost::filesystem::path mat_file = boost::filesystem::path(root_dir) / boost::filesystem::path(tree["mtl-file"].getValue<std::string>());
  if (!boost::filesystem::exists(mat_file)) throw(std::runtime_error("Error, the file doesn't exist!\n"));

  boost::filesystem::path tex_file = boost::filesystem::path(root_dir) / boost::filesystem::path(tree["texture"].getValue<std::string>());
  bool has_texture = false;
  if (!boost::filesystem::exists(tex_file)) throw(std::runtime_error("Error, the file doens't exist!\n"));
  
  if (has_texture = boost::filesystem::exists(tex_file)){
    ci::gl::Texture::Format format;
    target->GetTexture() = ci::gl::Texture::create(ci::loadImage((tex_file.string())), format);
  }

  ci::ObjLoader loader(ci::loadFile(obj_file.string()), ci::loadFile(mat_file.string()), true, true, true);
  target->GetMesh() = ci::gl::VboMesh::create(loader);


}

void Model::Draw() const {

  InternalDraw(internal_model_);

}

void Model::LoadData(const std::string &datafile_path){

  ci::JsonTree tree = OpenFile(datafile_path);

  LoadComponent(tree, internal_model_, boost::filesystem::path(datafile_path).parent_path().string());

}
