/*
 * @file talker.cpp
 * @author Rishabh Biyani (rishabh1b)
 * @copyright MIT License (c) Rishabh Biyani
 */

#include <math.h>
#include <sstream>
#include "ros/console.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "beginner_tutorials/StringService.h"
#include "beginner_tutorials/replaceString.h"
#include "tf/transform_broadcaster.h"

/**
* @brief method to replace the string being published
* @param a string to be replaced
*/

void replacePublishedString(std::string str) {
  curr_pub_string = str;
}

/**
* @brief service function which entertains the string changing calls
* @param Request is a type dependent of the service being used
* @return a boolean indicating whether service was completed correctly
*/

bool changeString(beginner_tutorials::replaceString::Request &req,
                  beginner_tutorials::replaceString::Response &res) {
  replacePublishedString(req.request_string);
  res.return_string = "The string is now changed to " + req.request_string;
  ROS_WARN("The output string just changed");
  return true;
}

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv) {
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "talker");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * Initializing some parameters that can set through command line or launch file
   */
  int rate;
  std::string topic;

  /**
   * Initialize node parameters from launch file or command line.
   * Use a private node handle so that multiple instances of the node can
   * be run simultaneously while using different parameters.
   */

  ros::NodeHandle private_node_handle_("~");
  private_node_handle_.param("rate", rate, static_cast<int>(40));
  private_node_handle_.param("topic", topic, std::string("chatter"));

  ros::ServiceServer service_1 = n.advertiseService("changeString",
                                                    changeString);
  ROS_INFO("String Replacing Service is now being provided");

  // Create a tf::transform object to be broadcasted
  int omega = 2;
  tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::Quaternion q;
  q.setRPY(0, 0 , M_PI/2);
  transform.setRotation(q);

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  ros::Publisher chatter_pub = n.advertise < std_msgs::String > (topic, 1000);

  ros::Rate loop_rate(rate);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  while (ros::ok()) {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    std_msgs::String msg;

    std::stringstream ss;
    ss << curr_pub_string << count;
    msg.data = ss.str();

    // ROS_INFO("%s", msg.data.c_str());

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    chatter_pub.publish(msg);
    // Sinusoidally change the origin of the frame and then broadcast
    transform.setOrigin(tf::Vector3(sin(omega * ros::Time::now().toSec()), cos(omega * ros::Time::now().toSec()), 0.0));
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/world", "/talk"));

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  return 0;
}

