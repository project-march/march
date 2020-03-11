// Copyright 2019 Project March.
#include "march_safety/input_device_safety.h"

#include <string>

InputDeviceSafety::InputDeviceSafety(ros::NodeHandle* n, SafetyHandler* safety_handler) : is_connected_(false)
{
  int milliseconds;
  ros::param::get("~input_device_connection_timeout", milliseconds);
  int send_errors_interval_param;
  ros::param::get("~send_errors_interval", send_errors_interval_param);
  this->send_errors_interval_ = send_errors_interval_param;
  this->connection_timeout_ = ros::Duration(milliseconds / 1000.0);
  this->safety_handler_ = safety_handler;
  this->time_last_alive_ = ros::Time(0);
  this->time_last_send_error_ = ros::Time(0);
  this->subscriber_input_device_alive_ = n->subscribe<std_msgs::Time>(
      "/march/input_device/alive", 1000, &InputDeviceSafety::inputDeviceAliveCallback, this);
}

void InputDeviceSafety::inputDeviceAliveCallback(const std_msgs::TimeConstPtr& msg)
{
  this->time_last_alive_ = msg->data;
}

void InputDeviceSafety::update(const ros::Time& now)
{
  if (this->time_last_alive_.toSec() == 0)
  {
    ROS_INFO_THROTTLE(5, "No input device connected yet");
    return;
  }

  // Check if there is no alive msg receive for the timeout duration.
  if (now > this->time_last_alive_ + this->connection_timeout_)
  {
    this->is_connected_ = false;
    std::ostringstream message_stream;
    message_stream << "Input Device Connection Lost. Current time is " << now.toSec() << " and last alive message was "
                   << this->time_last_alive_.toSec()
                   << ". The difference in time is: " << now.toSec() - this->time_last_alive_.toSec();
    std::string error_message = message_stream.str();

    this->sendError(now, error_message);
  }
  else if (!this->is_connected_)
  {
    this->is_connected_ = true;
    ROS_INFO("Input device reconnected");
  }

  // Check if the alive msg is not timestamped with a future time.
  // This can happen when one node is using sim_tim and others aren't.
  // Add small margin to take the stamp offset between board and PC into account
  if (now + ros::Duration(0.5) < this->time_last_alive_)
  {
    std::ostringstream message_stream;
    message_stream << "Input Device Connection message is from the future. Current time: " << now.toSec()
                   << " and last alive message was: " << this->time_last_alive_.toSec()
                   << "The difference in time is: " << this->time_last_alive_.toSec() - now.toSec();
    std::string error_message = message_stream.str();
    this->sendError(now, error_message);
  }
}

void InputDeviceSafety::sendError(const ros::Time& now, const std::string& message)
{
  if (now > this->time_last_send_error_ + ros::Duration(this->send_errors_interval_ / 1000.0))
  {
    this->safety_handler_->publishNonFatal(message);
    this->time_last_send_error_ = now;
  }
}
