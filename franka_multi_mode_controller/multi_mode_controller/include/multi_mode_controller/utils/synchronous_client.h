#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"

namespace panda_controllers {
template <typename Service>
class SynchronousClient : public rclcpp::Client<Service> {
 public:
  SynchronousClient(rclcpp::node_interfaces::NodeBaseInterface* node_base, 
      rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph, 
      const std::string& service_name, 
      rcl_client_options_t& client_options) : 
      rclcpp::Client<Service>(node_base, node_graph, service_name, 
                              client_options) {}
  typename Service::Response::SharedPtr callWithTimeout(
      const typename Service::Request::SharedPtr& request, 
      const double& timeout_s) {
    std::shared_future<typename Service::Response::SharedPtr> res_future = 
        this->async_send_request(request);
    std::future_status status = res_future.wait_for(
        std::chrono::duration<double>(timeout_s));
    if (status == std::future_status::ready) {
      return res_future.get();
    } else if (status == std::future_status::timeout) {
      RCLCPP_WARN(rclcpp::get_logger("SynchronousClient"), 
          "Service %s timed out.", this->get_service_name());
      return nullptr;
    } else {
      RCLCPP_WARN(rclcpp::get_logger("SynchronousClient"), 
          "Service %s was deferred.", this->get_service_name());
      return nullptr;
    }
  }
};

class SynchronousClientFactory : public rclcpp::Node {
 public:
  SynchronousClientFactory(const std::string& node_name, 
      const rclcpp::NodeOptions& options=rclcpp::NodeOptions());
  SynchronousClientFactory(const std::string& node_name,
      const std::string &namespace_, 
      const rclcpp::NodeOptions& options=rclcpp::NodeOptions());
  ~SynchronousClientFactory();
  template <typename Service>
  std::shared_ptr<SynchronousClient<Service>> create_synchronous_client(
      const std::string& service_name, 
      const rmw_qos_profile_t& qos_profile=rmw_qos_profile_services_default) {
    rcl_client_options_t client_options = rcl_client_get_default_options();
    auto client = std::make_shared<SynchronousClient<Service>>(
        this->get_node_base_interface().get(), this->get_node_graph_interface(), 
        service_name, client_options);
    auto cli_base_ptr = std::dynamic_pointer_cast<rclcpp::ClientBase>(client);
    this->get_node_services_interface()->add_client(cli_base_ptr, nullptr);
    return client;
  }
 private:
  void setupSpinner();
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread execution_thread_;
};

}