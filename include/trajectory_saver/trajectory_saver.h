#ifndef TRAJECTORY_SAVER_TRAJECTORY_SAVER_H
#define TRAJECTORY_SAVER_TRAJECTORY_SAVER_H

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <manipulation_msgs/srv/add_trajectory.hpp>
#include <manipulation_msgs/srv/get_trajectory.hpp>
#include <manipulation_msgs/srv/remove_trajectory.hpp>
#include <trajectory_saver/warehouse_store.hpp>

namespace trajectory_saver
{
class TrajectorySaver : public rclcpp::Node
{
public:
  explicit TrajectorySaver(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  void start();

private:
  void rosSetup();
  void addTrajectoryCallback(
    const manipulation_msgs::srv::AddTrajectory::Request::SharedPtr request,
    manipulation_msgs::srv::AddTrajectory::Response::SharedPtr response);
  void getTrajectoryCallback(
    const manipulation_msgs::srv::GetTrajectory::Request::SharedPtr request,
    manipulation_msgs::srv::GetTrajectory::Response::SharedPtr response);
  void removeTrajectoryCallback(
    const manipulation_msgs::srv::RemoveTrajectory::Request::SharedPtr request,
    manipulation_msgs::srv::RemoveTrajectory::Response::SharedPtr response);
  void amountOfTrajectoriesCallback(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response);

  std::string add_trajectory_service_name_ = "/robot/arm/trajectory_saver/add_trajectory";
  std::string get_trajectory_service_name_ = "/robot/arm/trajectory_saver/get_trajectory";
  std::string remove_trajectory_service_name_ = "/robot/arm/trajectory_saver/remove_trajectory";
  std::string amount_of_trajectories_service_name_ = "/robot/arm/trajectory_saver/amount_of_trajectories";
  bool interfaces_started_{false};

  std::unique_ptr<WarehouseStore> warehouse_store_;
  rclcpp::Service<manipulation_msgs::srv::AddTrajectory>::SharedPtr add_trajectory_service_;
  rclcpp::Service<manipulation_msgs::srv::GetTrajectory>::SharedPtr get_trajectory_service_;
  rclcpp::Service<manipulation_msgs::srv::RemoveTrajectory>::SharedPtr remove_trajectory_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr amount_of_trajectories_service_;
};
}

#endif