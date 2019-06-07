// Copyright 2018 Project March.

#include "ros/ros.h"
#include <sound_play/sound_play.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>

#include <march_sound_scheduler/Scheduler.h>

Scheduler* scheduler;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "march_sound_scheduler");
  ros::NodeHandle n;

  ros::Publisher pub_sound = n.advertise<sound_play::SoundRequest>("/robotsound", 0);

  while (0 == pub_sound.getNumSubscribers())
  {
    ROS_INFO("Waiting on sound play topic");
    ros::Duration(0.1).sleep();
  }

  sound_play::SoundClient sc;
  sc.playWaveFromPkg("march_sound_scheduler", "sounds/hi.ogg");
  scheduler = new Scheduler();

  ros::spin();

  return 0;
}
