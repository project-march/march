//
// Created by tim on 23-10-18.
//

#include "gtest/gtest.h"
#include "../src/march_main_node/TestCube.h"
#include <gmock/gmock.h>
#include "ros/ros.h"

class cube_test : public ::testing::Test {
 protected:
  TestCube test_cube = TestCube(10);

};

TEST_F(cube_test, TestVolume){
  ASSERT_EQ(10, test_cube.getVolume());
}

/**
 * The main method which runs all the tests
 */
int main(int argc, char **argv) {
  ::testing::InitGoogleMock(&argc, argv);
  ros::init(argc, argv, "tester");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}


