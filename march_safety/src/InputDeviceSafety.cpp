// Copyright 2019 Project March.
#include <march_safety/InputDeviceSafety.h>

InputDeviceSafety::InputDeviceSafety(ros::Publisher* error_publisher, ros::NodeHandle n)
{
  int milliseconds;
  n.getParam(ros::this_node::getName() + std::string("/input_device_connection_timeout"), milliseconds);
  this->connection_timeout = ros::Duration(0, milliseconds * 1000000);
  this->error_publisher = error_publisher;
  this->createSubscribers();
  this->time_last_alive = ros::Time(0);
}

void InputDeviceSafety::inputDeviceAliveCallback(const std_msgs::TimeConstPtr& msg)
{
  this->time_last_alive = msg->data;
}

march_shared_resources::Error InputDeviceSafety::createErrorMessage()
{
  march_shared_resources::Error error_msg;
  // @TODO(Tim) Come up with real error codes
  error_msg.error_code = 1;  // For now a randomly chosen error code
  error_msg.error_message = "Input Device Connection Lost";
  error_msg.type = march_shared_resources::Error::FATAL;
  return error_msg;
}

void InputDeviceSafety::createSubscribers()
{
  subscriber_input_device_alive = n.subscribe<std_msgs::Time>("/march/input_device/alive", 1000,
                                                              &InputDeviceSafety::inputDeviceAliveCallback, this);
}

void InputDeviceSafety::checkConnection()
{
  if (time_last_alive.toSec() == 0)
  {
    ROS_DEBUG_THROTTLE(5, "No input device connected yet");
    return;
  }
  if (ros::Time::now() > time_last_alive + this->connection_timeout)
  {
    auto error_msg = createErrorMessage();
    ROS_ERROR("%i, %s", error_msg.error_code, error_msg.error_message.c_str());
    error_publisher->publish(error_msg);
  }
}
