#pragma once

#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <warehouse_ros/database_loader.h>

namespace trajectory_saver
{
class WarehouseStore
{
public:
  using Trajectory = std_msgs::msg::String;

  /** @brief Connect to the SQLite warehouse and open the typed trajectory collection. */
  explicit WarehouseStore(const rclcpp::Node::SharedPtr & node)
  : loader_(node), logger_(node->get_logger())
  {
    const char * home = std::getenv("HOME");
    const auto default_path = std::filesystem::path(home ? home : ".") /
      ".ros" / "manipulation.sqlite";
    node->declare_parameter("warehouse_plugin", "warehouse_ros_sqlite::DatabaseConnection");
    node->declare_parameter("warehouse_port", 0);
    const auto path = node->declare_parameter("warehouse_host", default_path.string());
    const auto database = node->declare_parameter("database_name", "manipulation");
    if (path.empty() || database.empty()) {
      throw std::invalid_argument("warehouse_host and database_name must not be empty");
    }
    if (path != ":memory:" && !std::filesystem::path(path).parent_path().empty()) {
      std::filesystem::create_directories(std::filesystem::path(path).parent_path());
    }
    connection_ = loader_.loadDatabase();
    if (!connection_ || !connection_->setParams(path, 0) || !connection_->connect()) {
      throw std::runtime_error("Unable to connect to warehouse: " + path);
    }
    // Keep trajectory records independent from the pose collection's ROS schema.
    trajectories_ = connection_->openCollectionPtr<Trajectory>(database, "trajectories");
    if (!trajectories_->md5SumMatches()) {
      throw std::runtime_error("Warehouse message schema does not match this build");
    }
  }

  /** @brief Insert a trajectory string or replace its current stored revision. */
  bool putTrajectory(const std::string & name, const Trajectory & trajectory)
  {return put(trajectories_, name, trajectory);}

  /** @brief Load the newest stored revision into the output message. */
  bool getTrajectory(const std::string & name, Trajectory & trajectory)
  {return get(trajectories_, name, trajectory);}

  /** @brief Remove all stored revisions for a trajectory name. */
  bool removeTrajectory(const std::string & name) {return remove<Trajectory>(trajectories_, name);}

  /** @brief Count unique trajectory names rather than physical revisions. */
  std::size_t trajectoryCount() {return names<Trajectory>(trajectories_).size();}

private:
  template<class Message>
  using Collection = typename warehouse_ros::MessageCollection<Message>::Ptr;

  /** @brief Reject names that cannot be used as stable record keys. */
  static void validateName(const std::string & name)
  {
    if (name.empty() || name.find_first_not_of(" \t\r\n") == std::string::npos) {
      throw std::invalid_argument("Name must not be empty or whitespace");
    }
  }

  template<class Message>
  /** @brief Insert a new revision before attempting to clean older records. */
  bool put(const Collection<Message> & collection, const std::string & name, const Message & message)
  {
    validateName(name);
    auto query = collection->createQuery();
    query->append("name", name);
    // Insert the next revision before attempting old-record cleanup.
    const auto previous = collection->count() == 0 ?
      std::vector<typename warehouse_ros::MessageWithMetadata<Message>::ConstPtr>{} :
      collection->queryList(query, true, "revision", false);
    int revision = 1;
    if (!previous.empty()) {
      const int last = previous.front()->lookupInt("revision");
      if (last == std::numeric_limits<int>::max()) {
        throw std::overflow_error("Record revision exhausted");
      }
      revision = last + 1;
    }
    auto metadata = collection->createMetadata();
    metadata->append("name", name);
    metadata->append("revision", revision);
    collection->insert(message, metadata);
    if (!previous.empty()) {
      // A cleanup error must not turn a successful replacement into data loss.
      query->appendLT("revision", revision);
      try {
        collection->removeMessages(query);
      } catch (const std::exception & error) {
        RCLCPP_WARN(logger_, "Saved replacement; old revision cleanup failed: %s", error.what());
      }
    }
    return !previous.empty();
  }

  template<class Message>
  /** @brief Load the newest stored revision for a name, if present. */
  static bool get(const Collection<Message> & collection, const std::string & name, Message & message)
  {
    validateName(name);
    if (collection->count() == 0) {
      return false;
    }
    auto query = collection->createQuery();
    query->append("name", name);
    // Descending revision order returns the newest version first.
    const auto records = collection->queryList(query, false, "revision", false);
    if (records.empty()) {
      return false;
    }
    message = *records.front();
    return true;
  }

  template<class Message>
  /** @brief Remove all stored revisions matching a name. */
  static bool remove(const Collection<Message> & collection, const std::string & name)
  {
    validateName(name);
    if (collection->count() == 0) {
      return false;
    }
    auto query = collection->createQuery();
    query->append("name", name);
    return collection->removeMessages(query) > 0;
  }

  template<class Message>
  /** @brief Return unique metadata names in sorted order. */
  static std::vector<std::string> names(const Collection<Message> & collection)
  {
    std::vector<std::string> result;
    for (const auto & record : collection->queryList(collection->createQuery(), true)) {
      result.push_back(record->lookupString("name"));
    }
    // Multiple stored revisions may share a name if prior cleanup failed.
    std::sort(result.begin(), result.end());
    result.erase(std::unique(result.begin(), result.end()), result.end());
    return result;
  }

  warehouse_ros::DatabaseLoader loader_;
  rclcpp::Logger logger_;
  warehouse_ros::DatabaseConnection::Ptr connection_;
  Collection<Trajectory> trajectories_;
};
}