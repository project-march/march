// Copyright 2019 Project March.
#include <march_safety/InputDeviceSafety.h>

InputDeviceSafety::InputDeviceSafety(ros::Publisher* error_publisher, ros::NodeHandle n)
{
  int milliseconds;
  n.getParam(ros::this_node::getName() + std::string("/input_device_connection_timeout"), milliseconds);
  double send_errors_interval_param;
  n.getParam(ros::this_node::getName() + std::string("/send_errors_interval"), send_errors_interval_param);
  this->send_errors_interval = send_errors_interval_param;
  this->connection_timeout = ros::Duration(milliseconds / 1000);
  this->error_publisher = error_publisher;
  this->time_last_alive = ros::Time(0);
  this->time_last_send_error = ros::Time(0);
  this->subscriber_input_device_alive = n.subscribe<std_msgs::Time>("/march/input_device/alive", 1000,
                                                                    &InputDeviceSafety::inputDeviceAliveCallback, this);
}

void InputDeviceSafety::inputDeviceAliveCallback(const std_msgs::TimeConstPtr& msg)
{
  this->time_last_alive = msg->data;
}

march_shared_resources::Error InputDeviceSafety::createErrorMessage()
{
  march_shared_resources::Error error_msg;
  std::ostringstream message_stream;
  message_stream << "Input Device Connection Lost. Current time: " << ros::Time::now().toSec()
                 << " and last alive message was: " << this->time_last_alive.toSec()
                 << "The difference in time is: " << ros::Time::now().toSec() - this->time_last_alive.toSec();
  std::string error_message = message_stream.str();
  // @TODO(Tim) Come up with real error codes
  error_msg.error_code = 1;  // For now a randomly chosen error code
  error_msg.error_message = error_message;
  error_msg.type = march_shared_resources::Error::FATAL;
  return error_msg;
}

march_shared_resources::Error InputDeviceSafety::createFutureErrorMessage()
{
  march_shared_resources::Error error_msg;
  std::ostringstream message_stream;
  message_stream << "Input Device Connection message is from the future. Current time: " << ros::Time::now().toSec()
                 << " and last alive message was: " << this->time_last_alive.toSec()
                 << "The difference in time is: " << this->time_last_alive.toSec() - ros::Time::now().toSec();
  std::string error_message = message_stream.str();
  // @TODO(Tim) Come up with real error codes
  error_msg.error_code = 1;  // For now a randomly chosen error code
  error_msg.error_message = error_message;
  error_msg.type = march_shared_resources::Error::FATAL;
  return error_msg;
}

void InputDeviceSafety::checkConnection()
{
  if (time_last_alive.toSec() == 0)
  {
    ROS_DEBUG_THROTTLE(5, "No input device connected yet");
    return;
  }
  // send at most an error every second
  if (ros::Time::now() > time_last_send_error + ros::Duration(this->send_errors_interval / 1000))
  {
    // Check if there is no alive msg receive for the timeout duration.
    if (ros::Time::now() > time_last_alive + this->connection_timeout)
    {
      auto error_msg = createErrorMessage();
      ROS_ERROR("%i, %s", error_msg.error_code, error_msg.error_message.c_str());
      error_publisher->publish(error_msg);
      this->time_last_send_error = ros::Time::now();
    }
    // Check if the alive msg is not timestamped with a future time.
    // This can happen when one node is using sim_tim and others aren't.
    if (ros::Time::now() < time_last_alive)
    {
      auto error_msg = createFutureErrorMessage();
      ROS_ERROR("%i, %s", error_msg.error_code, error_msg.error_message.c_str());
      error_publisher->publish(error_msg);
      this->time_last_send_error = ros::Time::now();
    }
  }
}
