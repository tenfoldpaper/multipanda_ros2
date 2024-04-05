#include <multi_mode_controller/utils/robot_data.h>

#include <stdexcept>


using namespace panda_controllers;

RobotData::RobotData(RobotData&& rd) : frm_(std::move(rd.frm_)), 
    resource_(std::move(rd.resource_)) {}

RobotData::RobotData(const std::string& resource) : resource_(resource) {};

// The getters should probably return a reference, instead of a copy
const Eigen::Matrix<double, 7, 1>& RobotData::coriolis() {
  std::lock_guard<std::mutex> lock(update_mutex_);
  return coriolis_;
}

const Eigen::Matrix<double, 6, 7>& RobotData::eeZeroJacobian() {
  std::lock_guard<std::mutex> lock(update_mutex_);
  return ee_zero_jacobian_;
}

const Eigen::Matrix<double, 6, 7>& RobotData::eeBodyJacobian() {
  std::lock_guard<std::mutex> lock(update_mutex_);
  return ee_body_jacobian_;
}

const Eigen::Matrix<double, 7, 7>& RobotData::mass() {
  std::lock_guard<std::mutex> lock(update_mutex_);
  return mass_;
}

const franka::RobotState& RobotData::state() {
  std::lock_guard<std::mutex> lock(update_mutex_);
  return state_;
}

// Eigen::Matrix<double, 7, 1> RobotData::getCoriolis() {
//   std::lock_guard<std::mutex> lock(update_mutex_);
//   return coriolis_;
// }

// Eigen::Matrix<double, 6, 7> RobotData::getEeZeroJacobian() {
//   std::lock_guard<std::mutex> lock(update_mutex_);
//   return ee_zero_jacobian_;
// }

// Eigen::Matrix<double, 6, 7> RobotData::getEeBodyJacobian() {
//   std::lock_guard<std::mutex> lock(update_mutex_);
//   return ee_zero_jacobian_;
// }

// Eigen::Matrix<double, 7, 7> RobotData::getMass() {
//   std::lock_guard<std::mutex> lock(update_mutex_);
//   return mass_;
// }

// franka::RobotState RobotData::getState() {
//   std::lock_guard<std::mutex> lock(update_mutex_);
//   return state_;
// }

bool RobotData::init(std::string robot_model_interface_name,
                     std::string robot_state_interface_name) {
  std::lock_guard<std::mutex> lock(update_mutex_);
  try{
    auto model_name = resource_ + robot_model_interface_name;
    auto state_name = resource_ + robot_state_interface_name;
    frm_ = std::make_unique<franka_semantic_components::FrankaRobotModel>(
        franka_semantic_components::FrankaRobotModel(model_name,
                                                      resource_));

  }
  catch (std::runtime_error& e){
    std::cerr << "Robot data initialization failed!\n";
    return false;
  }

  return true;
}

bool RobotData::activate_model(std::vector<hardware_interface::LoanedStateInterface> &state_interfaces){
  std::lock_guard<std::mutex> lock(update_mutex_);
  // Takes the passed reference to the state interfaces and call assign_loaned_state_interfaces on the frm
  frm_->assign_loaned_state_interfaces(state_interfaces);
  // then initializes it
  try{
    frm_->initialize();
  }
  catch (std::exception& e){
    std::cerr << e.what() << std::endl;
    return false;
  }
  return true;
}
bool RobotData::deactivate_model(){
  std::lock_guard<std::mutex> lock(update_mutex_);
  frm_->release_interfaces();
  return true;
}

void RobotData::update() {
  std::lock_guard<std::mutex> lock(update_mutex_);
  
  if(frm_->update_state_and_model()){
    state_ = *frm_->getRobotState(); // might need to be fixed?
    coriolis_ = Eigen::Matrix<double, 7, 1>(frm_->getCoriolisForceVector().data());
    ee_zero_jacobian_ = Eigen::Matrix<double, 6, 7>(
        frm_->getZeroJacobian(franka::Frame::kEndEffector).data());
    ee_body_jacobian_ = Eigen::Matrix<double, 6, 7>(
        frm_->getBodyJacobian(franka::Frame::kEndEffector).data());
    mass_ = Eigen::Matrix<double, 7, 7>(frm_->getMassMatrix().data());
  }
  else{
    std::cout << "update state and model for " << this->resource_ << " failed " << std::endl;
  };
}
