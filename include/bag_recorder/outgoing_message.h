#pragma once

// Main ROS
#include <ros/ros.h>
#include <topic_tools/shape_shifter.h>

// std tools
#include <queue>
#include <string>

namespace bag_recorder
{
class OutgoingMessage
{
public:
  OutgoingMessage(std::string const&                  _topic,
                  topic_tools::ShapeShifter::ConstPtr _msg,
                  boost::shared_ptr<ros::M_string>    _connection_header,
                  ros::Time                           _time);

  std::string                         topic;
  topic_tools::ShapeShifter::ConstPtr msg;
  boost::shared_ptr<ros::M_string>    connection_header;
  ros::Time                           time;
};  // OutgoingMessage
using message_queue_t = std::queue<OutgoingMessage>;
};  // namespace bag_recorder