#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "rov2_core/core_node.hpp"

// Standalone entry point for the core runtime. Also available as a composable
// component (rov2_core::CoreNode) for launch-profile composition.
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor executor;
  auto node = std::make_shared<rov2_core::CoreNode>();
  executor.add_node(node->get_node_base_interface());

  // autostart drives the managed node to the active state without an external
  // lifecycle manager, so `ros2 run rov2_core rov2_core_node` is self-testable.
  if (node->get_parameter("autostart").as_bool()) {
    node->configure();
    node->activate();
  }

  executor.spin();
  rclcpp::shutdown();
  return 0;
}
