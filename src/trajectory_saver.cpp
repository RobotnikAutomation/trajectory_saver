#include <trajectory_saver/trajectory_saver.h>

#include <functional>

namespace trajectory_saver
{
TrajectorySaver::TrajectorySaver(const rclcpp::NodeOptions & options)
: Node("trajectory_saver_node", options)
{
}

/** @brief Initialize the warehouse and advertise interfaces once node ownership is established. */
void TrajectorySaver::start()
{
  if (interfaces_started_) {
    return;
  }
  rosSetup();
  interfaces_started_ = true;
}

/** @brief Initialize the warehouse connection and advertise trajectory services. */
void TrajectorySaver::rosSetup()
{
  warehouse_store_ = std::make_unique<WarehouseStore>(this->shared_from_this());
  add_trajectory_service_ = this->create_service<manipulation_msgs::srv::AddTrajectory>(
    add_trajectory_service_name_,
    std::bind(&TrajectorySaver::addTrajectoryCallback, this, std::placeholders::_1, std::placeholders::_2));
  get_trajectory_service_ = this->create_service<manipulation_msgs::srv::GetTrajectory>(
    get_trajectory_service_name_,
    std::bind(&TrajectorySaver::getTrajectoryCallback, this, std::placeholders::_1, std::placeholders::_2));
  remove_trajectory_service_ = this->create_service<manipulation_msgs::srv::RemoveTrajectory>(
    remove_trajectory_service_name_,
    std::bind(&TrajectorySaver::removeTrajectoryCallback, this, std::placeholders::_1, std::placeholders::_2));
  amount_of_trajectories_service_ = this->create_service<std_srvs::srv::Trigger>(
    amount_of_trajectories_service_name_,
    std::bind(&TrajectorySaver::amountOfTrajectoriesCallback, this,
      std::placeholders::_1, std::placeholders::_2));
  RCLCPP_INFO(this->get_logger(), "Trajectory saver ready using warehouse_ros SQLite");
}

/** @brief Store the opaque trajectory string under its requested name. */
void TrajectorySaver::addTrajectoryCallback(
  const manipulation_msgs::srv::AddTrajectory::Request::SharedPtr request,
  manipulation_msgs::srv::AddTrajectory::Response::SharedPtr response)
{
  try {
    std_msgs::msg::String trajectory;
    trajectory.data = request->trajectory;
    response->message = warehouse_store_->putTrajectory(request->name, trajectory) ? "Updated" : "Added";
    response->success = true;
  } catch (const std::exception & error) {
    response->success = false;
    response->message = error.what();
    RCLCPP_ERROR(this->get_logger(), "Trajectory storage operation failed: %s", error.what());
  }
}

/** @brief Retrieve the trajectory string for a named record, if present. */
void TrajectorySaver::getTrajectoryCallback(
  const manipulation_msgs::srv::GetTrajectory::Request::SharedPtr request,
  manipulation_msgs::srv::GetTrajectory::Response::SharedPtr response)
{
  try {
    std_msgs::msg::String trajectory;
    response->success = warehouse_store_->getTrajectory(request->name, trajectory);
    if (response->success) {
      response->trajectory = trajectory.data;
    }
  } catch (const std::exception & error) {
    response->success = false;
    response->trajectory.clear();
    RCLCPP_ERROR(this->get_logger(), "Trajectory retrieval failed: %s", error.what());
  }
}

/** @brief Remove all stored revisions for the requested trajectory name. */
void TrajectorySaver::removeTrajectoryCallback(
  const manipulation_msgs::srv::RemoveTrajectory::Request::SharedPtr request,
  manipulation_msgs::srv::RemoveTrajectory::Response::SharedPtr response)
{
  try {
    response->success = warehouse_store_->removeTrajectory(request->name);
  } catch (const std::exception & error) {
    response->success = false;
    RCLCPP_ERROR(this->get_logger(), "Trajectory removal failed: %s", error.what());
  }
}

/** @brief Report the number of unique trajectory names in the collection. */
void TrajectorySaver::amountOfTrajectoriesCallback(
  const std_srvs::srv::Trigger::Request::SharedPtr,
  std_srvs::srv::Trigger::Response::SharedPtr response)
{
  try {
    response->message = std::to_string(warehouse_store_->trajectoryCount());
    response->success = true;
  } catch (const std::exception & error) {
    response->success = false;
    response->message = error.what();
    RCLCPP_ERROR(this->get_logger(), "Trajectory count failed: %s", error.what());
  }
}
}