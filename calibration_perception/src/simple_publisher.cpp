/*
 * simple_publisher.cpp
 *
 *  Created on: May 30, 2017
 *      Author: benny
 */

#include "ros/ros.h"

#include <actionlib/server/simple_action_server.h>
#include <caltab_detector/FindCaltabAction.h>

class Example {
protected:
  ros::NodeHandle nh;
  actionlib::SimpleActionServer<caltab_detector::FindCaltabAction> actionServer;
  std::string actionName;
  caltab_detector::FindCaltabFeedback feedback;
  caltab_detector::FindCaltabResult result;
  ros::Rate rate;

public:
  ros::AsyncSpinner spinner;

public:
  Example(std::string name) : actionServer(nh, name, boost::bind(&Example::executeCB, this, _1), false),
                                 actionName(name), rate(10), spinner(4) {
    actionServer.start();
  }

  void executeCB(const caltab_detector::FindCaltabGoalConstPtr &goal) {
    bool success = false;
    int counter = 0;
    std::mt19937 rng;
    rng.seed(std::random_device()());
    while (counter < 9) {
      if (actionServer.isPreemptRequested() || !ros::ok()) {
        ROS_INFO("%s: Preempted", actionName.c_str());
        actionServer.setPreempted();
        break;
      }
      std::uniform_int_distribution<std::mt19937::result_type> dist10(0,9); // distribution in range [1, 6]

      if (dist10(rng) == -1) {
        success = true;
        break;
      }
      feedback.feedback = 0;
//      counter++;
    }
    if (success) {
      result.result = 1;
      ROS_INFO("%s: Succeeded", actionName.c_str());
      actionServer.setSucceeded(result);
    } else {
      result.result = 0;
      ROS_INFO("%s: Failed", actionName.c_str());
      actionServer.setAborted(result);
    }

  }

};

int main(int argc, char **argv) {
  ros::init(argc, argv, "image_publisher");
  Example ex("testAction");
  ex.spinner.start();

  std::getchar();

}
