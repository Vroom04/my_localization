#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>

class StateEstimator : public rclcpp::Node {
public:
  StateEstimator() : Node("state_estimator_node") {
    // Subscriber all'odometria filtrata
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odometry/filtered", 10,
      std::bind(&StateEstimator::odomCallback, this, std::placeholders::_1));

    // Publisher per la posa stimata
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/state_estimation/pose", 10);

    // Publisher per il Marker RViz
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/state_estimation/marker", 10);
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Pubblica la posa stimata
    auto pose_msg = geometry_msgs::msg::PoseStamped();
    pose_msg.header = msg->header;
    pose_msg.pose = msg->pose.pose;
    pose_pub_->publish(pose_msg);

    // Marker per l'orientamento
    auto arrow = visualization_msgs::msg::Marker();
    arrow.header = msg->header;
    arrow.ns = "state_estimation";
    arrow.id = 0;
    arrow.type = visualization_msgs::msg::Marker::ARROW;
    arrow.action = visualization_msgs::msg::Marker::ADD;
    arrow.pose = msg->pose.pose;
    arrow.scale.x = 1.0;  // lunghezza freccia
    arrow.scale.y = 0.2;
    arrow.scale.z = 0.2;
    arrow.color.r = 1.0;
    arrow.color.a = 1.0;

    // Traiettoria
    auto line = visualization_msgs::msg::Marker();
    line.header = msg->header;
    line.ns = "trajectory";
    line.id = 1;
    line.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line.action = visualization_msgs::msg::Marker::ADD;
    line.scale.x = 0.1;
    line.color.b = 1.0;
    line.color.a = 1.0;

    geometry_msgs::msg::Point p;
    p.x = msg->pose.pose.position.x;
    p.y = msg->pose.pose.position.y;
    trajectory_points_.push_back(p);
    line.points = trajectory_points_;

    marker_pub_->publish(arrow);
    marker_pub_->publish(line);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  std::vector<geometry_msgs::msg::Point> trajectory_points_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StateEstimator>());
  rclcpp::shutdown();
  return 0;
}