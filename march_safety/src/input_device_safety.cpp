// Copyright 2019 Project March.
#include "march_safety/input_device_safety.h"

#include <string>

InputDeviceSafety::InputDeviceSafety(ros::NodeHandle* n, SafetyHandler* safety_handler)
  : safety_handler_(safety_handler)
{
  int milliseconds;
  ros::param::get("~input_device_connection_timeout", milliseconds);
  this->connection_timeout_ = ros::Duration(milliseconds / 1000.0);
  this->subscriber_input_device_alive_ = n->subscribe<march_shared_resources::Alive>(
      "/march/input_device/alive", 10, &InputDeviceSafety::inputDeviceAliveCallback, this);
}

void InputDeviceSafety::inputDeviceAliveCallback(const march_shared_resources::AliveConstPtr& msg)
{
  this->last_alive_stamps_[msg->id] = msg->stamp;
}

void InputDeviceSafety::update(const ros::Time& now)
{
  if (this->last_alive_stamps_.empty())
  {
    ROS_INFO_THROTTLE(5, "No input device connected yet");
    return;
  }

  const bool had_connections = !this->connected_devices_.empty();

  for (const auto& last_alive_stamp : this->last_alive_stamps_)
  {
    const std::string& id = last_alive_stamp.first;
    const ros::Time& last_alive = last_alive_stamp.second;
    const bool timed_out = now > (last_alive + this->connection_timeout_);
    const bool is_connected = this->connected_devices_.find(id) != this->connected_devices_.end();

    if (is_connected && timed_out)
    {
      this->connected_devices_.erase(id);
      if (id == "crutch" && !this->connected_devices_.empty())
      {
        this->safety_handler_->publishNonFatal("Crutch input device lost");
        ROS_ERROR_STREAM("Input device `" << id << "` lost");
      }
      else
      {
        ROS_WARN_STREAM("Input device `" << id << "` lost");
      }
    }
    else if (!is_connected && !timed_out)
    {
      this->connected_devices_.insert(id);
      ROS_INFO_STREAM("Input device `" << id << "` reconnected. Total connected is "
                                       << this->connected_devices_.size());
    }

    // Check if the alive msg is not timestamped with a future time.
    // This can happen when one node is using sim_time and others aren't.
    // Add small margin to take the stamp offset between board and PC into account
    if (now + ros::Duration(0.5) < last_alive)
    {
      ROS_WARN_STREAM_THROTTLE(5.0, "Input device `" << id << "` alive message is from the future. Current time is "
                                                     << now.toSec() << " and last alive message was "
                                                     << last_alive.toSec());
    }
  }

  const bool has_connections = !this->connected_devices_.empty();
  if (had_connections && !has_connections)
  {
    this->safety_handler_->publishNonFatal("All input devices are lost");
  }

  if (!has_connections)
  {
    ROS_INFO_DELAYED_THROTTLE(5.0, "No input device connected");
  }
}
