#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/bool.hpp"

// Waypoint relay that hands off from TARE to FAR planner.
//
// During exploration:
//   /tare_way_point (from TARE) → /way_point (to local_planner)
//
// When TARE publishes exploration_finish=true:
//   - Stops relaying TARE waypoints
//   - Publishes initial robot position to /goal_point (for FAR)
//   - FAR then takes over /way_point directly
class ExplorationRelay : public rclcpp::Node {
public:
  ExplorationRelay() : Node("exploration_relay") {
    waypoint_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(
        "/way_point", 2);
    goal_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(
        "/goal_point", 2);

    tare_waypoint_sub_ =
        create_subscription<geometry_msgs::msg::PointStamped>(
            "/tare_way_point", 2,
            std::bind(&ExplorationRelay::tareWaypointCb, this,
                      std::placeholders::_1));

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/state_estimation", 5,
        std::bind(&ExplorationRelay::odomCb, this, std::placeholders::_1));

    finish_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/exploration_finish", 2,
        std::bind(&ExplorationRelay::finishCb, this, std::placeholders::_1));
  }

private:
  void odomCb(const nav_msgs::msg::Odometry::SharedPtr msg) {
    if (!have_initial_pos_) {
      initial_pos_ = msg->pose.pose.position;
      have_initial_pos_ = true;
      RCLCPP_INFO(get_logger(), "Recorded home position: (%.2f, %.2f, %.2f)",
                  initial_pos_.x, initial_pos_.y, initial_pos_.z);
    }
  }

  void tareWaypointCb(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
    if (!exploration_finished_) {
      waypoint_pub_->publish(*msg);
    }
  }

  void finishCb(const std_msgs::msg::Bool::SharedPtr msg) {
    if (msg->data && !exploration_finished_) {
      exploration_finished_ = true;
      RCLCPP_INFO(get_logger(),
                  "Exploration finished — sending home goal to FAR planner");

      if (have_initial_pos_) {
        geometry_msgs::msg::PointStamped goal;
        goal.header.frame_id = "map";
        goal.header.stamp = now();
        goal.point = initial_pos_;
        goal_pub_->publish(goal);
      } else {
        RCLCPP_ERROR(get_logger(),
                     "No initial position recorded, cannot send home goal");
      }
    }
  }

  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr waypoint_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr goal_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr
      tare_waypoint_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr finish_sub_;

  geometry_msgs::msg::Point initial_pos_;
  bool have_initial_pos_ = false;
  bool exploration_finished_ = false;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ExplorationRelay>());
  rclcpp::shutdown();
  return 0;
}
