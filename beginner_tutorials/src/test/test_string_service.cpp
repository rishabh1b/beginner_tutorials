/**
* @file test_string_service.cpp - file for testing the string service provided by talker node
* @author Rishabh Biyani(rishabh1b)
* @copyright (c) MIT License Rishabh Biyani 2017
*/

#include "ros/ros.h"
#include "ros/service_client.h"
#include "beginner_tutorials/replaceString.h"
#include "gtest/gtest.h"

std::shared_ptr<ros::NodeHandle> nh;

/**
* @brief gtest function which checks test string service by creating a client
*/
TEST(StringSuite, changeString) {
  ros::ServiceClient client = nh->serviceClient<beginner_tutorials::replaceString>(
                              "changeString");
  bool exists(client.waitForExistence(ros::Duration(3)));

  EXPECT_TRUE(exists);

  beginner_tutorials::replaceString srv;
  srv.request.request_string = "gtestROS";
  client.call(srv);

  EXPECT_EQ(srv.response.return_string, "The string is now changed to gtestROS");
}

/**
* @brief main() functon to call all the unit tests using Google Test framework
*/
int main(int argc, char** argv) {
ros::init(argc, argv, "string_replace_testing");
nh.reset(new ros::NodeHandle);
testing::InitGoogleTest(&argc, argv);
return RUN_ALL_TESTS();
}
