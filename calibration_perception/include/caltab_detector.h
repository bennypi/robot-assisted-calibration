/*
 * This file is part of the robot-assisted-calibration project.
 *
 * Copyright (C) 2017 Benny Prange <benny.prange@googlemail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <iostream>
#include <chrono>
#include <thread>

#include <ros/ros.h>

#include <actionlib/server/simple_action_server.h>

#include <calibration_executive/FindCalibrationObjectAction.h>
#include <calibration_executive/CalculateParametersAction.h>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>

#include "HalconCpp.h"

#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>

using namespace HalconCpp;

#ifndef CALTAB_DETECTOR_H
#define CALTAB_DETECTOR_H

class Detector {
protected:
  ros::NodeHandle n;
  actionlib::SimpleActionServer<calibration_executive::FindCalibrationObjectAction> findCaltabActionServer;
  actionlib::SimpleActionServer<calibration_executive::CalculateParametersAction> calibrateActionServer;
  calibration_executive::FindCalibrationObjectResult findCaltabResult;
  calibration_executive::CalculateParametersResult calculateParametersResult;
  int callbacksLeft = 0;
  bool caltabFound;
  std::vector<double> caltabPosition, caltabOrientation;
  ros::Rate rate;
  image_transport::ImageTransport it;
  image_transport::Subscriber sub;
  std::string imagePath, descriptionFilePath;

  void subscriberCallback(const sensor_msgs::Image::ConstPtr &msg);

  void findCaltabAction(const calibration_executive::FindCalibrationObjectGoalConstPtr &goal);

  void calibrateAction(const calibration_executive::CalculateParametersGoalConstPtr &goal);

  void writeImageToFile(const HObject &pic, HTuple &file) const;

  void
  createInitialParameters(double focal_length, double sensor_size_y, double sensor_size_x, int img_resolution_y,
                          int img_resolution_x, HTuple &cameraParameters);

public:
  HTuple cameraParameters, calibDataId, suitablePictureIDs;
  HTuple file;
  int imageWidth, imageHeight, counter;

  void init();

  Detector();

  void SensorMsgToHalconImage(const sensor_msgs::Image::ConstPtr &msg, unsigned char *pointer);

  void SensorMsgToHalconImage(const sensor_msgs::Image::ConstPtr &msg, unsigned char *blue, unsigned char *green,
                              unsigned char *red);

  void addCaltabToImages();
};

#endif // CALTAB_DETECTOR_H
