#include "ros/ros.h"
#include "std_msgs/String.h"
#include "../public/communication/TopicNames.h"
#include <unistd.h>  // sleep
#include "gtest/gtest.h"

// Publishers
ros::Publisher residentEventPublisher;

// Subscribers
ros::Subscriber eventTriggerSubscriber;

// Store received messages
std::vector<EventTrigger> receivedEventTriggers;

class MasterNodeTest : public ::testing::Test
{
protected:
  MasterNodeTest()
  {
  }

  virtual void SetUp()
  {
    receivedEventTriggers.clear();
  }

  virtual void TearDown()
  {
  }
};

/**
 * Sending an EventTrigger on topic 'resident_event' to the scheduler
 * should result in an EventTrigger published on 'event_trigger'
 */
TEST_F(MasterNodeTest, requestAssistant)
{
  // Wait until a message has been received
  ros::Rate loop_rate(10);
  loop_rate.sleep();
  ros::spinOnce();
  ROS_INFO("Done spinning");

  /*** ASSERTIONS BEGIN ***/

  // Check we received only one EventTrigger
  ASSERT_EQ(1, receivedEventTriggers.size());

  //  msg = receivedEventTriggers[0];
  //
  //  // Check that the message has the correct information
  //  ASSERT_EQ(EVENT_TRIGGER_MSG_TYPE_REQUEST, msg.msg_type);
  //  ASSERT_EQ(EVENT_TRIGGER_EVENT_TYPE_ASSISTANT, msg.event_type);
  //  ASSERT_EQ(EVENT_TRIGGER_RESULT_FAILURE, msg.result);
}

/**
 * Callback for the 'event_trigger' topic messages.
 * Simply logs the received message into a list.
 */
void eventTriggerCallback(EventTrigger msg)
{
  receivedEventTriggers.push_back(msg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "TestScheduler");
  ros::NodeHandle nodeHandle;
  ros::Rate loop_rate(10);

  // Advertise and subscribe to topics
  ros::Subscriber eventTriggerSubscriber =
      nodeHandle.subscribe<elderly_care_simulation::EventTrigger>(TopicNames::gait_input, 1000, eventTriggerCallback);

  // Run tests to see if we received messages as expected
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}