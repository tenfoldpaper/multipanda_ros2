#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"

using namespace std::chrono_literals;

namespace panda_controllers {

template <typename ServiceType>
class ExecutorNode : public rclcpp::Node {
 public:
  ExecutorNode(const std::string& service_name) : service_name_(service_name), 
      Node( service_name.find_last_of('/') != std::string::npos ?
            service_name.substr(service_name.find_last_of('/') + 1) + "_executor_node" :
            service_name + "_executor_node") {
    client_ptr_ = this->create_client<ServiceType>(service_name, 
      rmw_qos_profile_services_default);
    executor_.add_node(this->get_node_base_interface());
    auto spin = [&]() {
      executor_.spin();
    };
    execution_thread_ = std::thread(spin);
  }
  ~ExecutorNode() {
    execution_thread_.join();
  }
  bool callWithTimeout(typename ServiceType::Request::SharedPtr& request,
                       typename ServiceType::Response::SharedPtr& response,
                       double timeout_s) {
    std::shared_future<typename ServiceType::Response::SharedPtr> res_future = 
        client_ptr_->async_send_request(request);
    std::future_status status = res_future.wait_for(
        std::chrono::duration<double>(timeout_s));
    if (status == std::future_status::ready) {
      response = res_future.get();
      return true;
    } else if (status == std::future_status::timeout) {
      RCLCPP_WARN(this->get_logger(), "Service %s timed out.", 
          service_name_.c_str());
      return false;
    } else {
      RCLCPP_WARN(this->get_logger(), "Service %s was deferred.", 
          service_name_.c_str());
      return false;
    }
  }
 private:
  std::string service_name_;
  typename rclcpp::Client<ServiceType>::SharedPtr client_ptr_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread execution_thread_;
};
}   // namespace cb_group_demo
