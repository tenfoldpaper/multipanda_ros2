#include "franka_hardware/robot_sim.hpp"
#include <cstring>
#include <Eigen/Dense>

namespace franka_hardware{ 

bool RobotSim::populateIndices(){
  mjModel* m_ = franka_hardware_model_->getMjModel();
  // body index loop
  for(int n = 0; n < kNumberOfJoints+2; n++){
    std::string link_name = robot_name_ + "_link" + std::to_string(n);
    for(int i = 0; i < m_->nbody; i++){
      int subarray_len = 0;
      int max_len = m_->name_bodyadr[i+1] - m_->name_bodyadr[i];
      if(i == m_->nbody - 1){
          max_len = 30;
      }
      for(int j = m_->name_bodyadr[i]; j < m_->name_bodyadr[i] + max_len; j++){
        subarray_len++;
        if(m_->names[j] == '\0'){
            break;
        }
      }
      char *temp_cname = new char[subarray_len];
      std::memcpy(temp_cname, m_->names + m_->name_bodyadr[i], subarray_len);
      std::string temp_name(temp_cname);
      if(link_name == temp_name){
        link_indices_[n] = i;
        delete temp_cname;
        break;
      }
    }
  }
  // joint site loop
  for(int n = 0; n < kNumberOfJoints+2; n++){
    std::string site_name;
    if(n < kNumberOfJoints){
      site_name = robot_name_ + "_joint" + std::to_string(n+1) + "_site";
    }
    else if (n == kNumberOfJoints){
      site_name = robot_name_ + "_flange_site";
    }
    else{
      site_name = robot_name_ + "_ee_site";
    }
    
    for(int i = 0; i < m_->nsite; i++){
      int subarray_len = 0;
      int max_len = m_->name_siteadr[i+1] - m_->name_siteadr[i];
      if(i == m_->nsite - 1){
          max_len = 30;
      }
      for(int j = m_->name_siteadr[i]; j < m_->name_siteadr[i] + max_len; j++){
        subarray_len++;
        if(m_->names[j] == '\0'){
            break;
        }
      }
      
      char *temp_cname = new char[subarray_len];
      std::memcpy(temp_cname, m_->names + m_->name_siteadr[i], subarray_len);
      std::string temp_name(temp_cname);
      if(site_name == temp_name){
        joint_site_indices_[n] = i;
        delete temp_cname;
        break;
      }
    }
  }

  // joint index loop
  for(int n = 0; n < kNumberOfJoints; n++){
    std::string joint_name = robot_name_ + "_joint" + std::to_string(n+1);
    for(int i = 0; i < m_->njnt; i++){
      
      int subarray_len = 0;
      int max_len = m_->name_jntadr[i+1] - m_->name_jntadr[i];
      if(i == m_->njnt - 1){
          max_len = 30;
      }
      for(int j = m_->name_jntadr[i]; j < m_->name_jntadr[i] + max_len; j++){
        subarray_len++;
        if(m_->names[j] == '\0'){
            break;
        }
      }
      char *temp_cname = new char[subarray_len];
      std::memcpy(temp_cname, m_->names + m_->name_jntadr[i], subarray_len);
      std::string temp_name(temp_cname);

      if(joint_name == temp_name){
        joint_indices_[n] = i;
        joint_qvel_indices_[n] = m_->jnt_dofadr[i]; // for qvel (nv x 1) indexing
        joint_qpos_indices_[n] = m_->jnt_qposadr[i]; // for qpos (nq x 1) indexing
        delete temp_cname;
        break;
      }
    }
  }
  

  // Find the actuator indices for each of the robot's torque and velocity actuators
  for(int n = 0; n < kNumberOfJoints; n++){
    std::string joint_name = robot_name_ + "_joint" + std::to_string(n+1);
    for(int i = 0; i < m_->nu; i++){
      int subarray_len = 0;
      int max_len = m_->name_actuatoradr[i+1] - m_->name_actuatoradr[i];
      if(i == m_->nu - 1){
          max_len = 30;
      }
      for(int j = m_->name_actuatoradr[i]; j < m_->name_actuatoradr[i] + max_len; j++){
        subarray_len++;
        if(m_->names[j] == '\0'){
          break;
        }
      }
      char *temp_cname = new char[subarray_len];
      std::memcpy(temp_cname, m_->names + m_->name_actuatoradr[i], subarray_len);
      std::string temp_name(temp_cname);
      if(robot_name_ + "_act_trq" + std::to_string(n+1) == temp_name){
        act_trq_indices_[n] = i;
      }
      if(robot_name_ + "_act_vel" + std::to_string(n+1) == temp_name){
        act_vel_indices_[n] = i;
      }
      delete temp_cname;
    }
  }

  // Gripper
  if(has_gripper_){
    // gripper joint loop
    for(int n = 0; n < 2; n++){
      std::string joint_name = robot_name_ + "_finger_joint" + std::to_string(n+1); //left, then right
      for(int i = 0; i < m_->njnt; i++){
        int subarray_len = 0;
        int max_len = m_->name_jntadr[i+1] - m_->name_jntadr[i];
        if(i == m_->njnt - 1){
            max_len = 30;
        }
        for(int j = m_->name_jntadr[i]; j < m_->name_jntadr[i] + max_len; j++){
          subarray_len++;
          if(m_->names[j] == '\0'){
              break;
          }
        }
        char *temp_cname = new char[subarray_len];
        std::memcpy(temp_cname, m_->names + m_->name_jntadr[i], subarray_len);
        std::string temp_name(temp_cname);

        if(joint_name == temp_name){
          gripper_joint_indices_[n] = i;
          gripper_joint_qvel_indices_[n] = m_->jnt_dofadr[i]; // for qvel (nv x 1) indexing
          gripper_joint_qpos_indices_[n] = m_->jnt_qposadr[i]; // for qpos (nq x 1) indexing
          delete temp_cname;
          break;
        }
      }
    }
    // gripper act loop
    std::string act_name = robot_name_ + "_act_gripper";
    for(int i = 0; i < m_->nu; i++){
      int subarray_len = 0;
      int max_len = m_->name_actuatoradr[i+1] - m_->name_actuatoradr[i];
      if(i == m_->nu - 1){
          max_len = 30;
      }
      for(int j = m_->name_actuatoradr[i]; j < m_->name_actuatoradr[i] + max_len; j++){
        subarray_len++;
        if(m_->names[j] == '\0'){
          break;
        }
      }
      char *temp_cname = new char[subarray_len];
      std::memcpy(temp_cname, m_->names + m_->name_actuatoradr[i], subarray_len);
      std::string temp_name(temp_cname);
      if(act_name == temp_name){
        gripper_act_idx_ = i;
        break;
      }
      delete temp_cname;
    }
  }
  setModelIndices();
  return true;
}

franka::RobotState RobotSim::populateFrankaState(){
  /*
    Incomplete:
    std::array<double, 7UL> q_d_; // desired ...
    std::array<double, 7UL> dq_d_; // desired ...
    std::array<double, 7UL> ddq_d_; // desired ...
    std::array<double, 7UL> tau_J_d_; // desired ...
    std::array<double, 7UL> dtau_J_; // derivative joint torque
    
    franka::RobotState current_state_; // <- pack all of them into franka::RobotState
  */
  // mjModel* m = franka_hardware_model_->getMjModel();
  mjData* d = franka_hardware_model_->getMjData();
  double tau_J[7] = {0};
  for(int i=0; i<kNumberOfJoints; i++){
    current_state_.q[i] = d->qpos[joint_qpos_indices_[i]];
    current_state_.dq[i] = d->qvel[joint_qvel_indices_[i]];
    // the actual franka publishes non-zero values when in gravcomp mode, so add the qfrc_gravcomp to match that
    current_state_.tau_J[i] = d->actuator_force[act_trq_indices_[i]] + d->qfrc_gravcomp[act_trq_indices_[i]]; 
    tau_J[i] = d->actuator_force[act_trq_indices_[i]]; 
  }

  // calculate end effector jacobian, and then compose the EE force transform
  double basePos[3] = {0};
  double basePos_I[3] = {0};
  double baseQuat[4] = {0};
  double baseQuat_I[4] = {0};
  franka_hardware_model_->getXPosQuatbyLink(basePos, baseQuat, link_indices_[8]);
  mju_negPose(basePos_I, baseQuat_I, basePos, baseQuat);
  double forceTMat[36] = {0};
  franka_hardware_model_->composeForceTransform(forceTMat, basePos_I, baseQuat_I);
  // compose the 6*7
  double Jac[42] = {0};
  franka_hardware_model_->get6x7Jacobian(Jac, joint_site_indices_[8]);

  // multiply the jac
  double rJac[42] = {0};
  mju_mulMatMat(rJac, forceTMat, Jac, 6, 6, 7);
  // finally, get the force
  double eeForce[6] = {0};
  mju_mulMatVec(eeForce, rJac, tau_J, 6, 7);
  // forceTMat * Jac * torques
  for(int i=0; i<6; i++){
    // we want the end-effector external forces, so 8th body in the link indices.
    // This needs to be transformed into the base frame.
    current_state_.O_F_ext_hat_K[i] = eeForce[i];
  }

  // xpos is in W;
  // l7_W -> l7_B
  // T^B_W = l0_W
  // 
  // Doesn't really work; set up the inverses and stuff properly later.
  // o_t_ee_ = xpose[link0] * xpose[link7]
  int& s7 = joint_site_indices_[7];
  int& l0 = link_indices_[0];
  double eePosres[3] = {0};
  double eeQuatres[4] = {0};
  franka_hardware_model_->getBSiteInALinkframe(eePosres, eeQuatres, l0, s7);

  // transform data so that we can assign to O_T_EE
  double o_t_ee_rMVec[9] = {0};
  mju_quat2Mat(o_t_ee_rMVec, eeQuatres);
  double TMat[16] = {o_t_ee_rMVec[0], o_t_ee_rMVec[1], o_t_ee_rMVec[2], eePosres[0],
                     o_t_ee_rMVec[3], o_t_ee_rMVec[4], o_t_ee_rMVec[5], eePosres[1],
                     o_t_ee_rMVec[6], o_t_ee_rMVec[7], o_t_ee_rMVec[8], eePosres[2],
                     0,0,0,1};
  // finally, write to the state
  // convert to col major format
  // convertToColMajor
  int k = 0;
  for (int col = 0; col < 4; ++col) {
    for (int row = 0; row < 4; ++row) {
      current_state_.O_T_EE[k] = TMat[row * 4 + col];
      k++;
    }
  }

  return current_state_;
}

franka_hardware::ModelSim* RobotSim::getModel() {
      return franka_hardware_model_.get();
}

bool RobotSim::setModelIndices(){
  franka_hardware_model_->setIndices(link_indices_,
                                     joint_site_indices_,
                                     joint_indices_,
                                     joint_qpos_indices_,
                                     joint_qvel_indices_,
                                     act_trq_indices_,
                                     act_vel_indices_);
  return true;
}

} // namespace franka_hardware