#ifndef BT_ROS_LOGGER_HPP
#define BT_ROS_LOGGER_HPP

#include <cstring>
#include "behaviortree_cpp_v3/loggers/abstract_logger.h"


#include "rclcpp/rclcpp.hpp"

//#include "std_msgs/msg/bool.hpp"
#include "bt_msgs/msg/bt_transition.hpp"

namespace BT
{

class ROS_logger : public StatusChangeLogger
{
  static std::atomic<bool> ref_count;


private:
  std::shared_ptr<rclcpp::Node> node;
  rclcpp::executors::MultiThreadedExecutor exec;
  rclcpp::Publisher<bt_msgs::msg::BtTransition>::SharedPtr bt_visualization_publisher;
  


public:
  ROS_logger(const BT::Tree& tree);
  ~ROS_logger() override;

  virtual void callback(Duration timestamp, const TreeNode& node, NodeStatus prev_status,
                        NodeStatus status) override;

  virtual void flush() override;
};

}   // namespace BT

#endif   // BT_ROS_LOGGER_HPP
