// Copyright 2018 Project March.

#include "ros/ros.h"
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>

#include <march_sound_scheduler/Scheduler.h>
#include <march_shared_resources/Sound.h>

Scheduler* scheduler;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "march_sound_scheduler");
  ros::NodeHandle n;

  ros::Publisher pub_sound = n.advertise<sound_play::SoundRequest>("/robotsound", 0);
//  ros::Subscriber soundSub = n.subscribe<()

  while (0 == pub_sound.getNumSubscribers())
  {
    ROS_INFO("Waiting on sound play topic");
    ros::Duration(0.1).sleep();
  }

  scheduler = new Scheduler();

    scheduler->schedule(ScheduledSound(ros::Time::now() + ros::Duration(5), "r2-alarm.wav"));

  ros::Rate rate(10);
  while (ros::ok())
  {
    rate.sleep();
    ros::spinOnce();
    scheduler->spin();
  }

  return 0;
}
