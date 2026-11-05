#include <memory>
#include <trajectory_saver/trajectory_saver.h>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    auto node = std::make_shared<trajectory_saver::TrajectorySaver>();
    rclcpp::executors::MultiThreadedExecutor executor;
    node->start();
    executor.add_node(node);
    executor.spin();
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("trajectory_saver"), "%s", error.what());
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}