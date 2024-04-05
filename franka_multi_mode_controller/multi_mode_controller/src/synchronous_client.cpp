#include "multi_mode_controller/utils/synchronous_client.h"

namespace panda_controllers {

SynchronousClientFactory::SynchronousClientFactory(const std::string &node_name, 
    const rclcpp::NodeOptions& options) : Node(node_name, options) {
  setupSpinner();
}

SynchronousClientFactory::SynchronousClientFactory(const std::string &node_name, 
    const std::string &namespace_, const rclcpp::NodeOptions& options) :
    Node(node_name, namespace_, options) {
  setupSpinner();
}

SynchronousClientFactory::~SynchronousClientFactory() {
  execution_thread_.join();
}

void SynchronousClientFactory::setupSpinner() {
  executor_.add_node(this->get_node_base_interface());
  std::function<void()> spin = [&]() {
    executor_.spin();
  };
  execution_thread_ = std::thread(spin);
}

}