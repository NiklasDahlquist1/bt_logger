#include "bt_ros_logger/bt_ros_logger.hpp"

namespace BT
{
std::atomic<bool> ROS_logger::ref_count(false);

ROS_logger::ROS_logger(const BT::Tree& tree) : StatusChangeLogger(tree.rootNode())
{
  bool expected = false;
  if (!ref_count.compare_exchange_strong(expected, true))
  {
    throw LogicError("Only one instance of ROS_logger shall be created");
  }

  std::cout << "STARTING LOGGER\n";

    this->node = rclcpp::Node::make_shared("bt_ros_visualizer");
    this->bt_visualization_publisher = this->node->create_publisher<bt_msgs::msg::BtTransition>("bt_visualization/transitions", 20);

    this->exec.add_node(this->node);
}
ROS_logger::~ROS_logger()
{
  ref_count.store(false);
}

void ROS_logger::callback(Duration timestamp, const TreeNode& node,
                             NodeStatus prev_status, NodeStatus status)
{
  this->exec.spin_some();
  bt_msgs::msg::BtTransition msg;

  msg.node_name = node.name();
  msg.timestamp = this->node.get()->get_clock().get()->now();

  switch (prev_status)
  {
  case BT::NodeStatus::SUCCESS:
    msg.prev_status = bt_msgs::msg::BtTransition::SUCCESS;
    break;
  case BT::NodeStatus::RUNNING:
    msg.prev_status = bt_msgs::msg::BtTransition::RUNNING;
    break;  
  case BT::NodeStatus::FAILURE:
    msg.prev_status = bt_msgs::msg::BtTransition::FAILURE;
    break;
  case BT::NodeStatus::IDLE:
    msg.prev_status = bt_msgs::msg::BtTransition::IDLE;
    break;
  case BT::NodeStatus::SKIPPED:
    msg.prev_status = bt_msgs::msg::BtTransition::SKIPPED;
    break;
  default:
    break;
  }

  switch (status)
  {
  case BT::NodeStatus::SUCCESS:
    msg.status = bt_msgs::msg::BtTransition::SUCCESS;
    break;
  case BT::NodeStatus::RUNNING:
    msg.status = bt_msgs::msg::BtTransition::RUNNING;
    break;  
  case BT::NodeStatus::FAILURE:
    msg.status = bt_msgs::msg::BtTransition::FAILURE;
    break;
  case BT::NodeStatus::IDLE:
    msg.status = bt_msgs::msg::BtTransition::IDLE;
    break;
  case BT::NodeStatus::SKIPPED:
    msg.status = bt_msgs::msg::BtTransition::SKIPPED;
    break;
  default:
    break;
  }

  


  this->bt_visualization_publisher.get()->publish(msg);


  // using namespace std::chrono;

  // constexpr const char* whitespaces = "                         ";
  // constexpr const size_t ws_count = 25;

  // double since_epoch = duration<double>(timestamp).count();
  // printf("[%.3f]: %s%s %s -> %s", since_epoch, node.name().c_str(),
  //        &whitespaces[std::min(ws_count, node.name().size())],
  //        toStr(prev_status, true).c_str(), toStr(status, true).c_str());
  // std::cout << std::endl;
}

void ROS_logger::flush()
{
  // std::cout << std::flush;
  ref_count = false;
}

}   // namespace BT
