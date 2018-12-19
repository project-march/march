#include "gtest/gtest.h"

class SquareTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
  }
};

TEST_F(SquareTest, TestArea)
{
  ASSERT_EQ(36, 10);
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  //  ros::init(argc, argv, "test_talker");
  //  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}