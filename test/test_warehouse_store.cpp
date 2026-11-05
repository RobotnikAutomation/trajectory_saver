#include <gtest/gtest.h>
#include <trajectory_saver/warehouse_store.hpp>
#include <chrono>

TEST(WarehouseStore, SQLiteCrudAndPersistence)
{
  rclcpp::init(0, nullptr);
  const auto path = std::filesystem::temp_directory_path() /
    ("trajectory_saver_test_" + std::to_string(
      std::chrono::steady_clock::now().time_since_epoch().count()) + ".sqlite");
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("warehouse_host", path.string())});
  auto node = std::make_shared<rclcpp::Node>("warehouse_store_test", options);
  using Store = trajectory_saver::WarehouseStore;
  {
    Store store(node);
    Store::Trajectory trajectory;
    trajectory.data = "movej([0,1,2,3,4,5])";
    EXPECT_FALSE(store.putTrajectory("home_tf", trajectory));
    EXPECT_EQ(store.trajectoryCount(), 1u);
    trajectory.data = "replacement";
    EXPECT_TRUE(store.putTrajectory("home_tf", trajectory));
    EXPECT_EQ(store.trajectoryCount(), 1u);
    EXPECT_THROW(store.putTrajectory("   ", trajectory), std::invalid_argument);
    EXPECT_FALSE(store.getTrajectory("missing", trajectory));
  }
  node.reset();
  node = std::make_shared<rclcpp::Node>("warehouse_store_reopen_test", options);
  {
    Store store(node);
    Store::Trajectory trajectory;
    ASSERT_TRUE(store.getTrajectory("home_tf", trajectory));
    EXPECT_EQ(trajectory.data, "replacement");
    EXPECT_TRUE(store.removeTrajectory("home_tf"));
    EXPECT_FALSE(store.removeTrajectory("missing"));
    EXPECT_EQ(store.trajectoryCount(), 0u);
  }
  node.reset();
  rclcpp::shutdown();
  std::filesystem::remove(path);
}