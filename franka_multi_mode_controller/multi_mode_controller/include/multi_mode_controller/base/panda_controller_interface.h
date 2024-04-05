#pragma once

#include <vector>

// includes for measurements
#include <time.h>
#include <sys/time.h>
#include <fstream>

#include <multi_mode_controller/utils/robot_data.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
namespace panda_controllers {

struct tau_measurement{
  tau_measurement(const std::vector<std::array<double,7>> tau, double wall_time) 
    : tau_(tau), wall_time_(wall_time){};
  std::vector<std::array<double,7>> tau_;
  double wall_time_;
};

class PandaControllerInterface {
 public:
  virtual ~PandaControllerInterface() = default;
  bool hasError() {
    return error_;
  }
  bool hasOffset() {
    return hasOffset_();
  }
  bool init(const std::vector<RobotData*>& robot_data, rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
            std::string name, std::string resource) {
    // NodeHandle could be handled differently, since it just deals with namespace stuff
    name_ = name;
    node_ = node;
    resource_ = resource;
    robot_data_ = robot_data;
    cycle_count_ = 0;
    tau_msmt_.reserve(max_count_);
    cycle_time_.reserve(max_count_);

    return init_(robot_data, node, name, resource);
  }
  bool moving() {
    return moving_();
  }
  void start() {start_();};
  void stop() {stop_();};
  void computeTau(const std::vector<std::array<double, 7>*>& tau) {
    return computeTau_(tau);
  }

  // for measurements
  double get_wall_time(){
        struct timeval time;
        if (gettimeofday(&time,NULL)){
            //  Handle error
            return 0;
        }
        return (double)time.tv_sec + (double)time.tv_usec * .000001;
  }
  double get_cpu_time(){
      return (double)clock() / CLOCKS_PER_SEC;
  }
  void measureTime(double& time_diff){
    cycle_time_[cycle_count_] = time_diff;
  };
  void measureTau(const std::vector<std::array<double,7>>& tau, double end_time){
    tau_msmt_[cycle_count_] = tau_measurement(tau, end_time);
  };
  void increaseCounter(){
    cycle_count_++;
  }
  int getCounter(){
    return cycle_count_;
  }
  void write_tau_to_file(std::string file_name){
    std::ofstream logfile;
    logfile.open(file_name + "_tau.txt");
    // populate header
    logfile << std::fixed << "time,"
            << "arm,"
            << "tau1,"
            << "tau2,"
            << "tau3,"
            << "tau4,"
            << "tau5,"
            << "tau6,"
            << "tau7\n";
    for(int i=0; i<cycle_count_;i++){
      for(size_t j=0; j<tau_msmt_[i].tau_.size(); j++){
        logfile << tau_msmt_[i].wall_time_ << "," 
                << j                       << ","
                << tau_msmt_[i].tau_[j][0] << ","
                << tau_msmt_[i].tau_[j][1] << ","
                << tau_msmt_[i].tau_[j][2] << ","
                << tau_msmt_[i].tau_[j][3] << ","
                << tau_msmt_[i].tau_[j][4] << ","
                << tau_msmt_[i].tau_[j][5] << ","
                << tau_msmt_[i].tau_[j][6] << "\n";
      }
    }
    logfile.close();
  }
  void write_time_to_file(std::string file_name){
    std::ofstream logfile;
    logfile.open(file_name + "_time.txt");
    logfile << std::fixed;
    for(int i=0; i<cycle_count_;i++){
      logfile << cycle_time_[i] << "\n"; 
    }
    logfile.close();
  }
  // for measurements

 protected:
  std::string name_;
  std::string resource_;
  std::vector<double> cycle_time_;
  std::vector<tau_measurement> tau_msmt_;
  std::vector<RobotData*> robot_data_;
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  bool error_;
  int cycle_count_;
  const int max_count_ = 30000;
  bool logged_ = false;
 private:
  virtual void computeTau_(const std::vector<std::array<double, 7>*>& tau) = 0;
  virtual bool hasOffset_() = 0;
  virtual bool init_(const std::vector<RobotData*>& robot_data, rclcpp_lifecycle::LifecycleNode::SharedPtr& node, std::string name, std::string resource) = 0;
  virtual bool moving_() = 0;
  virtual void start_() = 0;
  virtual void stop_() = 0;
};

}
