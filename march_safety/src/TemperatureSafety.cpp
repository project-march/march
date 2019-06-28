// Copyright 2019 Project March.
#include <march_safety/TemperatureSafety.h>

TemperatureSafety::TemperatureSafety(ros::Publisher* error_publisher, ros::Publisher* sound_publisher,
                                     ros::NodeHandle n)
{
  n.getParam(ros::this_node::getName() + std::string("/default_temperature_threshold"), default_temperature_threshold);
  n.getParam(ros::this_node::getName() + "/temperature_thresholds", temperature_thresholds_map);
  this->error_publisher = error_publisher;
  this->sound_publisher = sound_publisher;
  double send_errors_interval_param;
  n.getParam(ros::this_node::getName() + std::string("/send_errors_interval"), send_errors_interval_param);
  this->send_errors_interval = send_errors_interval_param;
  this->time_last_send_error = ros::Time(0);
  this->createSubscribers();
}

void TemperatureSafety::temperatureCallback(const sensor_msgs::TemperatureConstPtr& msg, const std::string& sensor_name)
{
  // send at most an error every second
  if (ros::Time::now() > time_last_send_error + ros::Duration(this->send_errors_interval / 1000))
  {
    // If the threshold is exceeded raise an error
    if (msg->temperature > getThreshold(sensor_name))
    {
      auto error_msg = createErrorMessage(msg->temperature, sensor_name);
      ROS_ERROR("%i, %s", error_msg.error_code, error_msg.error_message.c_str());
      error_publisher->publish(error_msg);
      march_shared_resources::Sound sound;
      sound.time = ros::Time::now();
      sound.file_name = "fatal.wav";
      sound_publisher->publish(sound);
      time_last_send_error = ros::Time::now();
    }
  }
}

double TemperatureSafety::getThreshold(const std::string& sensor_name)
{
  if (temperature_thresholds_map.find(sensor_name) != temperature_thresholds_map.end())
  {
    // Return specific defined threshold for this sensor
    return temperature_thresholds_map[sensor_name];
  }
  else
  {
    // Fall back to default if there is no defined threshold
    ROS_WARN_ONCE("There is no specific temperature threshold for %s sensor", sensor_name.c_str());
    return default_temperature_threshold;
  }
}

march_shared_resources::Error TemperatureSafety::createErrorMessage(double temperature, const std::string& sensor_name)
{
  march_shared_resources::Error error_msg;
  std::ostringstream message_stream;
  message_stream << sensor_name << " temperature too high: " << temperature;
  std::string error_message = message_stream.str();
  // @TODO(Tim) Come up with real error codes
  error_msg.error_code = 1;  // For now a randomly chosen error code
  error_msg.error_message = error_message;
  error_msg.type = march_shared_resources::Error::FATAL;
  return error_msg;
}

void TemperatureSafety::createSubscribers()
{
  std::vector<std::string> sensor_names;
  n.getParam("/sensors", sensor_names);
  for (const std::string& sensor_name : sensor_names)
  {
    // Use boost::bind to pass on the sensor_name as extra parameter to the callback method
    ros::Subscriber subscriber_temperature = n.subscribe<sensor_msgs::Temperature>(
        std::string(TopicNames::temperature) + "/" + sensor_name, 1000,
        boost::bind(&TemperatureSafety::temperatureCallback, this, _1, sensor_name));

    temperature_subscribers.push_back(subscriber_temperature);
  }
}
