#pragma once

#include <rclcpp/rclcpp.hpp>
#include <mujoco/mujoco.h>
#include <vector>
#include <iostream>
#include <string>
#include <algorithm>

#include <franka_msgs/srv/set_mujoco_poses.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
namespace franka_hardware {

struct MujocoObject{
  std::string obj_name_;
  double x;
  double y;
  double z;
  double qx;
  double qy;
  double qz;
  double qw;
};

class MujocoPoseServiceServer : public rclcpp::Node {
public:
  MujocoPoseServiceServer(const rclcpp::NodeOptions& options, std::vector<std::string> obj_names):
    rclcpp::Node("mujoco_pose_service_server", options), obj_names_(obj_names){
    
    has_updates_ = false;
    objects_.clear();
    set_mujoco_poses_service_ = create_service<franka_msgs::srv::SetMujocoPoses>(
      "~/set_mujoco_poses",
      std::bind(  // NOLINT [modernize-avoid-bind]
          &MujocoPoseServiceServer::setMujocoPosesCallback, this, std::placeholders::_1,
          std::placeholders::_2));
    RCLCPP_INFO(get_logger(), "mujoco_pose_service_server started");
  };
  bool hasUpdates(){
    return has_updates_;
  };
  void setHasUpdatesToFalse(){ // another class shouldn't be able to change has_updates to true
    has_updates_ = false;
  }
  std::vector<MujocoObject> getNewObjectPoses(){
    return objects_;
  };
private:
  bool has_updates_;
  std::vector<MujocoObject> objects_;
  std::vector<std::string> obj_names_;
  rclcpp::Service<franka_msgs::srv::SetMujocoPoses>::SharedPtr set_mujoco_poses_service_;
  void setMujocoPosesCallback(const franka_msgs::srv::SetMujocoPoses::Request::SharedPtr& request,
                       const franka_msgs::srv::SetMujocoPoses::Response::SharedPtr& response){
    //
    has_updates_ = false;
    bool has_updates = false;
    objects_.clear();
    int i = 0;
    for(auto object : request->objects){
      std::cout << object.header.frame_id << std::endl;
      if(std::find(obj_names_.begin(), obj_names_.end(), object.header.frame_id) != obj_names_.end()){
        response->valid.push_back(true);
        MujocoObject new_obj;
        new_obj.obj_name_ = object.header.frame_id;
        new_obj.x = object.pose.position.x;
        new_obj.y = object.pose.position.y;
        new_obj.z = object.pose.position.z;

        new_obj.qx = object.pose.orientation.x;
        new_obj.qy = object.pose.orientation.y;
        new_obj.qz = object.pose.orientation.z;
        new_obj.qw = object.pose.orientation.w;

        has_updates = true;
        objects_.push_back(new_obj);
      }
      else{
        response->valid.push_back(false);
      }
      i++;
    }
    // this at the end to ensure that the poses don't get set before it is fully populated
    has_updates_ = has_updates;
  };


};
}