#include <cinder/ObjLoader.h>
#include <cinder/gl/Texture.h>
#include <cinder/app/App.h>
#include <cinder/ImageIo.h>

#include <srvlib/model/node.hpp>
#include <srvlib/srvlib.hpp>
#include <srvlib/utils/math.hpp>

using namespace srvlib;

Node::~Node(){ }

void Node::LoadMeshAndTexture(ci::JsonTree &tree, const std::string &root_dir){

  boost::filesystem::path obj_file = boost::filesystem::path(root_dir) / boost::filesystem::path(tree["obj-file"].getValue<std::string>());
  if (!boost::filesystem::exists(obj_file)) throw(std::runtime_error("Error, the file doesn't exist!\n"));

  boost::filesystem::path mat_file = boost::filesystem::path(root_dir) / boost::filesystem::path(tree["mtl-file"].getValue<std::string>());
  if (!boost::filesystem::exists(mat_file)) throw(std::runtime_error("Error, the file doesn't exist!\n"));

  boost::filesystem::path texture_file = boost::filesystem::path(root_dir) / boost::filesystem::path(tree["tex-file"].getValue<std::string>());
  if (!boost::filesystem::exists(texture_file)) throw(std::runtime_error("Error, the file doesn't exist!\n"));

  ci::ObjLoader loader(ci::loadFile(obj_file.string()), ci::loadFile(mat_file.string()), true, true, true);
  mesh_ = ci::gl::VboMesh::create(loader);

  texture_ = ci::gl::Texture::create( ci::loadImage( texture_file.string()));

}


void Node::RenderTexture(int id){
  
  if (drawing_flag_){

    ci::gl::pushModelView();

    ci::gl::multModelMatrix(GetRelativeTransformToRoot()); 

    if (texture_){
      if (id == 0)
        texture_->bind();
      else
        texture_->bind(id);

    }

    if (mesh_->getNumVertices() != 0)
      ci::gl::draw(mesh_);

    if (texture_){
      texture_->unbind();
    }


    ci::gl::popModelView();
  }

  for (size_t i = 0; i < children_.size(); ++i){
    children_[i]->RenderTexture(id);
  }

}

std::vector< Node::Ptr > Node::GetAllChildren() {

  std::vector<Node::Ptr> all_children = children_;
  for (auto &child : children_){
    auto cc = child->GetAllChildren();
    all_children.insert(all_children.end(), cc.begin(), cc.end());
  }
  return all_children;

}

std::vector< Node::ConstPtr > Node::GetAllChildren() const{
  
  std::vector<Node::ConstPtr> all_children;
  for (auto &child : children_){
    all_children.push_back(child);
    auto cc = child->GetAllChildren();
    all_children.insert(all_children.end(), cc.begin(), cc.end());
  }
  return all_children;

}

std::vector<Node::ConstPtr> Node::GetChildren() const {

  std::vector<Node::ConstPtr> ret;
  std::copy(children_.cbegin(), children_.cend(), ret.begin());
  return ret;

}

Node::Ptr Node::GetChildByIdx(const std::size_t target_idx){

  if (idx_ == target_idx) return self_; //return shared from this

  for (size_t i = 0; i < children_.size(); ++i){
    auto c = children_[i]->GetChildByIdx(target_idx);
    if (c != nullptr) return c;
  }

  return nullptr;

}

Node::ConstPtr Node::GetChildByIdx(const std::size_t target_idx) const{

  if (idx_ == target_idx) return self_;

  for (size_t i = 0; i < children_.size(); ++i){
    auto c = children_[i]->GetChildByIdx(target_idx);
    if (c != nullptr) return c;
  }

  return nullptr;

}

void Node::ComputeJacobianForPoint(const glm::mat4 &world_transform, const glm::vec3 &point, const int target_frame_index, std::vector<glm::vec3> &jacobian) const {

  int idx = idx_;

  //the node with null parent is the root node so it's pose is basically just the base pose.
  if (parent_ != nullptr && idx_ != 3){ //3 is the fixed node so there's no derivative for it.

    //derivative of transform k = 
    //T_1 = transform from frame which point resides to parent of this frame (i.e. closest frame to point)
    //z = rotation axis of point
    //T_3 = transfrom from this frame to origin - with GetRelativeTransform

    if (!NodeIsChild(target_frame_index)){// || !NodeIsTransformable() ){

      jacobian.push_back(glm::vec3(0.0f, 0.0f, 0.0f));

    }
    else{

      //get the transform from this node to the node where the target point lives
      glm::mat4 T_1 = GetRelativeTransformToChild(target_frame_index);

      glm::mat4 T_3 = GetRelativeTransformToRoot();// *world_transform; //world transform has already been applied to point!

      //parent to node transform is in T_3. does not need to be applied again.
      glm::vec4 z;
      z = glm::vec4(GetAxis(), 1);
      
      glm::vec4 end = T_3 * glm::vec4(point, 1);
      
      glm::vec3 c = glm::cross(project(z), project(end));
      glm::vec4 jac = T_1 * unproject(c);

      jacobian.push_back(glm::vec3(jac[0], jac[1], jac[2]));

    }

  }

  for (size_t i = 0; i < children_.size(); ++i){
    children_[i]->ComputeJacobianForPoint(world_transform, point, target_frame_index, jacobian);
  }

}

bool Node::NodeIsChild(const size_t child_idx) const {

  auto c = GetChildByIdx(child_idx);
  if (c == nullptr) return false;
  else return true;

}

bool DHNode::NodeIsTransformable() const {

  return type_ == JointType::Rotation || type_ == JointType::Translation || type_ == JointType::Alternative;

}

void DHNode::GetPose(std::vector<float> &pose) const {

  if (parent_ != nullptr && NodeIsTransformable()){
    pose.push_back(update_);
  }

  for (size_t i = 0; i < children_.size(); ++i){
    children_[i]->GetPose(pose);
  }

}

void DHNode::ComputeJacobianForPoint(const glm::mat4 &world_transform, const glm::vec3 &point, const int target_frame_index, std::vector<glm::vec3> &jacobian) const {

  Node::ComputeJacobianForPoint(world_transform, point, target_frame_index, jacobian);
  return;

  //if (target_frame_index == 0){

  //  jacobian[7 + idx_] = glm::vec3(0.0f,0.0f,0.0f);

  //}

  //else if (target_frame_index == 1){

  //  ComputeJacobianForHead(world_transform, point, jacobian);

  //}

  //else if (target_frame_index == 4 || target_frame_index == 5){

  //  ComputeJacobianForClasperYaw(world_transform, point, jacobian);
  //  ComputeJacobianForClasperRotate(world_transform, point, target_frame_index, jacobian);

  //}
  //
  //else{

  //  jacobian[7 + idx_] = glm::vec3(0.0f, 0.0f, 0.0f);

  //}

  //for (size_t i = 0; i < children_.size(); ++i){
  //  children_[i]->ComputeJacobianForPoint(world_transform, point, target_frame_index, jacobian);
  //}

}

//void DHNode::ComputeJacobianForHead(const glm::mat4 &world_transform, const glm::vec3 &point, std::vector<glm::vec3> &jacobian) const{
//
//  if (!NodeIsChild(2)){
//
//    assert(idx_ == 2 || idx_ == 3 || idx_ == 4 || idx_ == 5);
//    jacobian[7 + idx_] = glm::vec3(0.0f, 0.0f, 0.0f);
//
//  }
//  else{
//
//    if (idx_ == 1){
//
//      //get point in the head's coordiante system
//      glm::mat4 transform_to_joint = world_transform;
//      //glhMultMatrixRight(GetRelativeTransformToRoot(), transform_to_joint);
//      transform_to_joint = transform_to_joint * GetRelativeTransformToRoot();
//
//      //get transform to the parent
//      glm::mat4 transform_to_joint_parent = world_transform;
//      //glhMultMatrixRight(parent_->GetRelativeTransformToRoot(), transform_to_joint_parent);
//      transform_to_joint_parent = transform_to_joint_parent * parent_->GetRelativeTransformToRoot();
//
//      const glm::mat4 derivative_transform = GetDerivativeTransfromFromParent();
//
//      const glm::vec4 point_in_joint_coords = glm::inverse(transform_to_joint) * unproject(point);
//
//      const glm::vec4 jac = transform_to_joint_parent * derivative_transform * point_in_joint_coords;
//
//      if (jac[3] != 1.0f) throw std::runtime_error("Error, bad jac");
//
//      jacobian[7 + idx_] = project(jac);
//
//    }
//
//  }
//
//}
//
//void DHNode::ComputeJacobianForClasperYaw(const glm::mat4 &world_transform, const glm::vec3 &point, std::vector<glm::vec3> &jacobian) const{
//
//  if (!NodeIsChild(3)){
//
//    assert(idx_ == 3 || idx_ == 4 || idx_ == 5);
//
//    jacobian[7 + idx_] = glm::vec3(0.0f, 0.0f, 0.0f);
//
//  }
//
//  if (idx_ == 1 || idx_ == 2){
//
//
//    glm::mat4 transform_to_joint = world_transform;
//    glhMultMatrixRight(GetRelativeTransformToRoot(), transform_to_joint);
//
//    //get transform to the parent
//    glm::mat4 transform_to_joint_parent = world_transform;
//    glhMultMatrixRight(parent_->GetRelativeTransformToRoot(), transform_to_joint_parent);
//
//    const glm::mat4 derivative_transform = GetDerivativeTransfromFromParent();
//
//    const glm::vec4 point_in_joint_coords = transform_to_joint.inverted() * point;
//
//    const glm::vec4 jac = transform_to_joint_parent * derivative_transform * point_in_joint_coords;
//
//    jacobian[7 + idx_] = jac.xyz();
//
//    //glm::vec4 jac = world_transform * parent_->GetRelativeTransformToRoot() * glm::vec4(dz.xyz(), 0.0f);
//    //jacobian[7 + idx_] = glm::vec3(jac[0], jac[1], jac[2]);
//      
//  }
//
//}
//
//void DHNode::ComputeJacobianForClasperRotate(const glm::mat4 &world_transform, const glm::vec3 &point, const int target_frame_index, std::vector<glm::vec3> &jacobian) const {
//
//  if (target_frame_index == 5 && idx_ == 4){
//    jacobian[7 + idx_] = glm::vec3(0.0f, 0.0f, 0.0f);
//    return;
//  }
//
//  if (target_frame_index == 4 && idx_ == 5){
//    jacobian[7 + idx_] = glm::vec3(0.0f, 0.0f, 0.0f);
//    return;
//  }
//
//  if (idx_ == 4 || idx_ == 5){ //ignore 3
//
//    glm::mat4 transform_to_joint = world_transform;
//    glhMultMatrixRight(GetRelativeTransformToRoot(), transform_to_joint);
//
//    //get transform to the parent
//    glm::mat4 transform_to_joint_parent = world_transform;
//    glhMultMatrixRight(parent_->GetRelativeTransformToRoot(), transform_to_joint_parent);
//
//    const glm::mat4 derivative_transform = GetDerivativeTransfromFromParent();
//
//    const glm::vec4 point_in_joint_coords = transform_to_joint.inverted() * point;
//
//    const glm::vec4 jac = transform_to_joint_parent * derivative_transform * point_in_joint_coords;
//
//    jacobian[7 + idx_] = jac.xyz();
//
//    //glm::vec4 jac = world_transform * parent_->GetRelativeTransformToRoot() * glm::vec4(dz.xyz(), 0.0f);
//    //jacobian[7 + idx_] = glm::vec3(jac[0], jac[1], jac[2]);
//
//  }
//  
//}

void DHNode::SetPose(std::vector<float>::iterator &pose){

  if (parent_ != nullptr && NodeIsTransformable()){
    update_ = *pose;
    ++pose;
  }

  for (size_t i = 0; i < children_.size(); ++i){
    children_[i]->SetPose(pose);
  }


}

void DHNode::UpdatePose(std::vector<float>::iterator &updates){
 
  //the node with null parent is the root node so it's pose is basically just the base pose and therefore there is not an update
  if (parent_ != nullptr && NodeIsTransformable()){

    update_ += *updates;
    ++updates;  

  }

  for (size_t i = 0; i < children_.size(); ++i){
    children_[i]->UpdatePose(updates);
  }

}

glm::mat4 DHNode::GetRelativeTransformToNodeByIdx(const int target_idx) const{
  
  //for nodes that are parents of the target node just return identity
  if (target_idx > idx_){
    return glm::mat4();
  }
  //before we get to the target node, return the transform to the parent * recursive call up the chain to parent
  else{
    if (parent_ != 0x0){
      DHNode::Ptr p = boost::dynamic_pointer_cast<DHNode>(parent_);
      //return glhMultMatrixRight(GetTransformFromParent(), p->GetRelativeTransformToNodeByIdx(target_idx));
      return p->GetRelativeTransformToNodeByIdx(target_idx) * GetTransformFromParent();
    }
    else{
      glm::mat4 m;
      return m;
    }
  }
  

}

glm::mat4 DHNode::GetRelativeTransformToChild(const int child_idx) const {

  if (child_idx == idx_)
    return glm::mat4();
  else
    return GetChildByIdx(child_idx)->GetRelativeTransformToNodeByIdx(idx_);

}

glm::mat4 DHNode::GetWorldTransform(const glm::mat4 &base_frame_transform) const {
  
  if (parent_ != 0x0){
    DHNode::Ptr p = boost::dynamic_pointer_cast<DHNode>(parent_);
    //return glhMultMatrixRight(GetTransformFromParent(), p->GetWorldTransform(base_frame_transform));
    return p->GetWorldTransform(base_frame_transform) * GetTransformFromParent();
  }
  else{ 
    return base_frame_transform;
  }

}

glm::mat4 DHNode::GetRelativeTransformToRoot() const {

  if (parent_ != 0x0){
    DHNode::Ptr p = boost::dynamic_pointer_cast<DHNode>(parent_);
    return p->GetRelativeTransformToRoot() * GetTransformFromParent();
  }
  else{
    glm::mat4 m;
    return m;// GetTransformToParent();
  }

}

void DHNode::LoadData(ci::JsonTree &tree, Node::Ptr parent, Node::Ptr self, const std::string &root_dir, size_t &idx){
  
  //set and increase the index
  idx_ = idx;
  idx++;

  //set the parent
  if (parent != 0x0){
    parent_ = boost::dynamic_pointer_cast<DHNode>(parent);  }
  else{
    parent_ = 0x0;
  }

  self_ = boost::dynamic_pointer_cast<DHNode>(self);

  //load the visual data (texture, vertices etc)
  try{
    LoadMeshAndTexture(tree, root_dir);
  }
  catch (ci::JsonTree::Exception &){

  }

  update_ = 0.0;

  //set up the dh param stuff
  try{
    
    alpha_ = tree["dh"]["alpha"].getValue<float>();
    theta_ = tree["dh"]["theta"].getValue<float>();
    a_ = tree["dh"]["a"].getValue<float>();
    d_ = tree["dh"]["d"].getValue<float>();
    
    if (tree["dh"]["type"].getValue<std::string>() == "rotation")
      type_ = Rotation;
    else if (tree["dh"]["type"].getValue<std::string>() == "translation")
      type_ = Translation;
    else if (tree["dh"]["type"].getValue<std::string>() == "fixed")
      type_ = Fixed;
    else
      throw std::runtime_error("Error, bad value in JSON file.");

    alt_axis_ = glm::vec3(0, 0, 0);

  }
  catch (ci::JsonTree::Exception &){

    try{

      ci::JsonTree rotate_axis = tree.getChild("rotate");
      alt_axis_[0] = rotate_axis[0].getValue<float>();
      alt_axis_[1] = rotate_axis[1].getValue<float>();
      alt_axis_[2] = rotate_axis[2].getValue<float>();
      type_ = Alternative;

    }
    catch (ci::JsonTree::Exception &){

      type_ = Fixed;

    }

    //should just give the identity transform for this node (useful e.g. for the root node).
    alpha_ = (float)M_PI / 2;
    theta_ = (float)M_PI / 2;
    a_ = 0.0f;
    d_ = 0.0f;
    

  }

  ci::JsonTree children = tree.getChild("children");
  for (size_t i = 0; i < children.getChildren().size(); ++i){
    Node::Ptr n(new DHNode);
    n->LoadData(children[i], self_, n, root_dir, idx);
    AddChild(n);
  }

}

void DHNode::createFixedTransform(const glm::vec3 &axis, const float rads, glm::mat4 &output) const {

  output = glm::rotate(rads, axis);
  //output = glm::mat4::createRotation(axis, rads);

}

void alternativeDerivativeTransform(const glm::vec3 &axis, const float theta, glm::mat4 &deriv){

  deriv = glm::mat4();

  if (axis == glm::vec3(0, 1, 0)) {
    deriv[0][0] = -sin(theta);
    deriv[2][0] = cos(theta);
    deriv[0][2] = -cos(theta);
    deriv[2][2] = -sin(theta);
  }
  else if (axis == glm::vec3(0, -1, 0)){
    deriv[0][0] = -sin(theta);
    deriv[2][0] = -cos(theta);
    deriv[0][2] = cos(theta);
    deriv[2][2] = -sin(theta);
  }
  else{
    throw std::runtime_error("");
  }
}

glm::mat4 DHNode::GetDerivativeTransfromFromParent() const {

  glm::mat4 DH;

  if (type_ == Translation)
    math::glhDenavitHartenbergDerivative(a_ * SCALE, alpha_, (d_ + update_) * SCALE, theta_, DH);
  else if (type_ == Rotation)
    math::glhDenavitHartenbergDerivative(a_ * SCALE, alpha_, d_ * SCALE, theta_ + update_, DH);
  else if (type_ == Fixed)
    math::glhDenavitHartenbergDerivative(a_ * SCALE, alpha_, d_ * SCALE, theta_, DH);
  else if (type_ == Alternative)
    alternativeDerivativeTransform(alt_axis_, theta_, DH);

  return DH;

}

glm::mat4 DHNode::GetTransformFromParent() const {
  
  glm::mat4 DH;
  
  if (type_ == Translation)
    math::glhDenavitHartenberg(a_ * SCALE, alpha_, (d_ + update_) * SCALE, theta_, DH);
  else if (type_ == Rotation)
    math::glhDenavitHartenberg(a_ * SCALE, alpha_, d_ * SCALE, theta_ + update_, DH);
  else if (type_ == Fixed)
    math::glhDenavitHartenberg(a_ * SCALE, alpha_, d_ * SCALE, theta_, DH);
  else if (type_ == Alternative)
    createFixedTransform(alt_axis_, update_, DH);

  return DH;

}
