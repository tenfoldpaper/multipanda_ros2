#include "franka_hardware/mobile_base_sim.hpp"
#include <cstring>

namespace franka_hardware{ 

bool MobileBaseSim::populateIndices(mjModel* m_){
//   mjModel* m_ = franka_model->getMjModel();
  // body index loop
  for(int n = 0; n < kNumberOfJoints; n++){
    std::string link_name = wheel_names_[n] + "_wheel";
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
//   for(int n = 0; n < kNumberOfJoints+2; n++){
//     std::string site_name;
//     if(n < kNumberOfJoints){
//       site_name = robot_name_ + "_joint" + std::to_string(n+1) + "_site";
//     }
//     else if (n == kNumberOfJoints){
//       site_name = robot_name_ + "_flange_site";
//     }
//     else{
//       site_name = robot_name_ + "_ee_site";
//     }
    
//     for(int i = 0; i < m_->nsite; i++){
//       int subarray_len = 0;
//       int max_len = m_->name_siteadr[i+1] - m_->name_siteadr[i];
//       if(i == m_->nsite - 1){
//           max_len = 30;
//       }
//       for(int j = m_->name_siteadr[i]; j < m_->name_siteadr[i] + max_len; j++){
//         subarray_len++;
//         if(m_->names[j] == '\0'){
//             break;
//         }
//       }
      
//       char *temp_cname = new char[subarray_len];
//       std::memcpy(temp_cname, m_->names + m_->name_siteadr[i], subarray_len);
//       std::string temp_name(temp_cname);
//       if(site_name == temp_name){
//         joint_site_indices_[n] = i;
//         std::cout << temp_name << std::endl;
//         delete temp_cname;
//         break;
//       }
//     }
//   }

  // joint index loop
  for(int n = 0; n < kNumberOfJoints; n++){
    std::string joint_name = wheel_names_[n] + "_wheel";
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
    std::string joint_name = wheel_names_[n] + "_wheel";
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
    //   if(robot_name_ + "_act_trq" + std::to_string(n+1) == temp_name){
    //     std::cout << temp_name << std::endl;
    //     act_trq_indices_[n] = i;
    //   }
    //   if(robot_name_ + "_act_pos" + std::to_string(n+1) == temp_name){
    //     std::cout << temp_name << std::endl;
    //     act_pos_indices_[n] = i;
    //   }
      if(joint_name + "_act_vel" == temp_name){
        act_vel_indices_[n] = i;
      }
      delete temp_cname;
    }
  }
  return true;
}
} // namespace franka_hardware