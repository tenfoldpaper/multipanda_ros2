#pragma once

#include <iostream>
#include <map>
#include <string>
#include <array>
// #include <franka/robot_state.h>
#include "mujoco/mujoco.h"
#include "franka_hardware/model_sim.hpp"
// #include "franka_hardware/control_mode.h"

namespace franka_hardware{
  class GarmiMujocoHardwareInterface; // for friending

class MobileBaseSim{
public:
    MobileBaseSim() = delete;
    MobileBaseSim(std::string& robot_name) : robot_name_(robot_name){}
    ~MobileBaseSim();
    bool populateIndices(mjModel* m_);

private:
    std::string robot_name_;
    std::array<int, 2UL> link_indices_; // for nbody indexing
    // std::array<int, 2UL> joint_site_indices_; // the joint sites positioned at the root acc. to the DH param 
    std::array<int, 2UL> joint_indices_; // for njnt indexing
    std::array<int, 2UL> joint_qpos_indices_; // for qpos (nq x 1) indexing
    std::array<int, 2UL> joint_qvel_indices_; // for qvel (nv x 1) indexing
    std::array<int, 2UL> act_vel_indices_; // part of nu

    // bool setModelIndices();
    const int kNumberOfJoints = 2;
    const std::array<std::string, 2UL> wheel_names_{"left","right"};

    friend class GarmiMujocoHardwareInterface;
    
};
} // namespace franka_hardware