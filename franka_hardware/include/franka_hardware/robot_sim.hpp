#pragma once

#include <iostream>
#include <map>
#include <string>
#include <array>
#include <franka/robot_state.h>
#include "franka_hardware/model_sim.hpp"
#include "franka_hardware/control_mode.h"

namespace franka_hardware{
  class FrankaMujocoHardwareInterface; // for friending

class RobotSim{
public:
    RobotSim() = delete;
    RobotSim(std::string& robot_name, bool has_gripper) : robot_name_(robot_name), has_gripper_(has_gripper) {}
    ~RobotSim();
    franka_hardware::ModelSim* getModel();
    bool populateIndices();
    franka::RobotState populateFrankaState();

private:
    std::string robot_name_;
    std::array<int, 9UL> link_indices_; // for nbody indexing
    std::array<int, 9UL> joint_site_indices_; // the joint sites positioned at the root acc. to the DH param 
    std::array<int, 7UL> joint_indices_; // for njnt indexing
    std::array<int, 7UL> joint_qpos_indices_; // for qpos (nq x 1) indexing
    std::array<int, 7UL> joint_qvel_indices_; // for qvel (nv x 1) indexing
    std::array<int, 7UL> act_trq_indices_; // part of nu
    std::array<int, 7UL> act_vel_indices_; // part of nu

    std::array<int, 2UL> gripper_joint_indices_;
    std::array<int, 2UL> gripper_joint_qpos_indices_;
    std::array<int, 2UL> gripper_joint_qvel_indices_;
    
    int gripper_act_idx_;

    bool setModelIndices();

    /* Franka State parameters */
    franka::RobotState current_state_; // <- pack all of them into franka::RobotState
    // and many more... to be included at a later time

    /* Franka Model parameters */
    std::array<double, 16> pose_;
    std::array<double, 49> mass_;
    std::array<double, 7> qfrc_bias_; // coriolis + gravity
    std::array<double, 42> zeroJacobian_;
    std::array<double, 42> bodyJacobian_;
    std::unique_ptr<ModelSim> franka_hardware_model_; // 
    const int kNumberOfJoints = 7;
    ControlMode control_mode_;
    bool has_gripper_;

    friend class FrankaMujocoHardwareInterface;
    
};
} // namespace franka_hardware