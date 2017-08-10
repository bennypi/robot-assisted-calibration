//TODO: Chose a license. Suggestions: GPLv3+ , LGPLv3+, Apache, BSD, ...
//TODO: Add copyright header

#include <caltab_detector.h>

Detector::Detector() :
    findCaltabActionServer(n, "find_calibration_object", boost::bind(&Detector::findCaltabAction, this, _1), false),
    calibrateActionServer(n, "calculate_parameters", boost::bind(&Detector::calibrateAction, this, _1), false),
    rate(5),
    it(n) {
}

void Detector::init() {
  // get values from the parameter server
  double focal_length, sensor_size_y, sensor_size_x;
  int resolution_x, resolution_y;
  std::string imageTopic;
  n.getParam("/calibration_perception/image_path", imagePath);
  n.getParam("/calibration_perception/focal_length", focal_length);
  n.getParam("/calibration_perception/sensor_size/y", sensor_size_y);
  n.getParam("/calibration_perception/sensor_size/x", sensor_size_x);
  n.getParam("/calibration_perception/resolution/x", resolution_x);
  n.getParam("/calibration_perception/resolution/y", resolution_y);
  n.getParam("/calibration_perception/image_topic", imageTopic);
  n.getParam("/calibration_perception/description_file", descriptionFilePath);

  // subscribe to the image topic
  sub = it.subscribe(imageTopic, 1, &Detector::subscriberCallback, this);

  // create the path to save the images
  imagePath.append("/calibration-images-");
  // get current date
  std::time_t t = std::time(nullptr);
  std::tm tm = *std::localtime(&t);
  std::ostringstream date_stream;
  date_stream << std::put_time(&tm, "%Y-%m-%d-%H-%M");
  std::string date = date_stream.str();
  // append date to path
  imagePath.append(date);
  imagePath.append("/");
  boost::filesystem::create_directories(imagePath);
  ROS_INFO_STREAM("Saving images to: " << imagePath.c_str());

  file.Append(imagePath.c_str());
  counter = 0;
  caltabFound = false;

  // Create action server
  findCaltabActionServer.start();
  calibrateActionServer.start();

  // create halcon objects
  try {
    CreateCalibData("calibration_object", 1, 1, &calibDataId);
    ROS_INFO_STREAM("Reading the calibration description file from: " << descriptionFilePath.c_str());
    SetCalibDataCalibObject(calibDataId, 0, HTuple(descriptionFilePath.c_str()));
    createInitialParameters(focal_length, sensor_size_x, sensor_size_y, resolution_x, resolution_y, cameraParameters);
    SetCalibDataCamParam(calibDataId, 0, "", cameraParameters);
  } catch (HOperatorException &e) {
    std::cout << e.ErrorMessage() << std::endl;
  }
  ROS_INFO("caltab_detector started");


}

void Detector::subscriberCallback(const sensor_msgs::Image::ConstPtr &msg) {
  // only process images if actiongoal is active
  if (callbacksLeft < 1) {
    return;
  }
  HImage pic;
  HTuple calibObjPose;
  try {
    // Create the different pixel arrays
    unsigned char pointer[msg->height * msg->width];
    unsigned char blue[msg->height * msg->width];
    unsigned char green[msg->height * msg->width];
    unsigned char red[msg->height * msg->width];

    // Depending on the message encoding create RGB or gray HImage
    if (strcmp(msg->encoding.c_str(), "bgr8") == 0) {
      SensorMsgToHalconImage(msg, blue, green, red);
      GenImage3(&pic, "byte", imageWidth, imageHeight, (long) red, (long) green, (long) blue);
    } else {
      SensorMsgToHalconImage(msg, pointer);
      GenImage1(&pic, "byte", imageWidth, imageHeight, (long) pointer);
    }

    // Find Caltab in image
    FindCalibObject(pic, calibDataId, 0, 0, counter, HTuple(), HTuple());
    GetCalibDataObservPose(calibDataId, 0, 0, counter, &calibObjPose);

    caltabPosition.clear();
    caltabOrientation.clear();
    for (int i = 0; i < 3; i++) {
      caltabPosition.push_back(calibObjPose[i].D());
    }
    for (int i = 3; i < 6; i++) {
      caltabOrientation.push_back(calibObjPose[i].D());
    }

    // Write pictures with found caltab to filesystem
    writeImageToFile(pic, file);

    // Store index for later use
    suitablePictureIDs = suitablePictureIDs.TupleConcat(counter);

    // caltab found, no more callbacks needed
    callbacksLeft = 0;
    caltabFound = true;
    ROS_INFO_STREAM("Caltab found and image saved.");
  } catch (cv_bridge::Exception &e) {
    ROS_ERROR_STREAM(e.what());
  } catch (HOperatorException &e) {
    std::string error = e.ErrorMessage().Text();
    // errorcode 8399 means image too blurry
    // errorcode 8397 means no calibration object found
    ROS_ERROR_STREAM(error);
  }
  counter++;
  callbacksLeft--;
}

void Detector::SensorMsgToHalconImage(const sensor_msgs::Image::ConstPtr &msg,
                                      unsigned char *pointer) {
  cv_bridge::CvImagePtr cv_ptr;
  std::__cxx11::string encoding;
  // Cannot convert ir-pictures to MONO8
  if (strcmp(msg->encoding.c_str(), "bgr8") == 0) {
    encoding = sensor_msgs::image_encodings::MONO8;
  } else {
    encoding = sensor_msgs::image_encodings::TYPE_8UC1;
  }

  // Store msg as cv::Mat
  cv_ptr = cv_bridge::toCvCopy(msg, encoding);
  imageWidth = cv_ptr->image.cols;
  imageHeight = cv_ptr->image.rows;

  // Create HALCON image from cv::Mat
  int i, j;
  uchar *p;
  for (i = 0; i < imageHeight; ++i) {
    p = cv_ptr->image.ptr<uchar>(i);
    for (j = 0; j < imageWidth; ++j) {
      pointer[i * imageWidth + j] = p[j];
    }
  }
}

void Detector::SensorMsgToHalconImage(const sensor_msgs::Image::ConstPtr &msg,
                                      unsigned char *blue, unsigned char *green, unsigned char *red) {
  cv_bridge::CvImagePtr cv_ptr;

  // Store msg as cv::Mat
  cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  imageWidth = cv_ptr->image.cols;
  imageHeight = cv_ptr->image.rows;

  int i, j;
  uchar *p;
  for (i = 0; i < imageHeight; ++i) {
    p = cv_ptr->image.ptr<uchar>(i);
    for (j = 0; j < imageWidth; ++j) {
      blue[i * imageWidth + j] = cv_ptr->image.at<cv::Vec3b>(i, j)[0];
      green[i * imageWidth + j] = cv_ptr->image.at<cv::Vec3b>(i, j)[1];
      red[i * imageWidth + j] = cv_ptr->image.at<cv::Vec3b>(i, j)[2];
    }
  }
}

void Detector::writeImageToFile(const HObject &pic, HTuple &file) const {
  HTuple path;
  path.Append(file);
  path.operator+=(counter);
  path.operator+=(".jpg");
  try {
    WriteImage(pic, "jpeg 100", 0, path);
  } catch (HOperatorException &e) {
    ROS_ERROR_STREAM("Error while saving the file " << path.ToString());
    ROS_ERROR_STREAM(e.ErrorMessage());
  }
}

void Detector::addCaltabToImages() {
  HalconCpp::HTuple root(file);
  HalconCpp::HTuple hv_CameraParam;

  // Read new calibration data
  GetCalibData(calibDataId, "camera", 0, "params", &hv_CameraParam);
  ROS_INFO_STREAM("Adding simulated caltab to every saved image.");

  // Read every saved file and create a new one with the simulated caltab
  for (int n = 0; n < suitablePictureIDs.Length(); n++) {
    long index = suitablePictureIDs[n];
    HalconCpp::HTuple hv_Pose, hv_save_filename;
    HalconCpp::HObject image, ho_simImage, ho_simAmp, ho_simDir, ho_Edges, ho_addedImage;

    ReadImage(&image, (root + HalconCpp::HTuple(index)) + ".jpg");

    try {
      //load caltab pose from the CalibDataID object
      GetCalibData(calibDataId, "calib_obj_pose", HTuple(0).TupleConcat(index), "pose", &hv_Pose);

      SimCaltab(&ho_simImage, descriptionFilePath.c_str(), hv_CameraParam, hv_Pose, 0, 80, 224, 1);
      EdgesImage(ho_simImage, &ho_simAmp, &ho_simDir, "canny", 1.0, "nms", 20, 40);
      Threshold(ho_simAmp, &ho_Edges, 1, 255);

      //This one paints only the edges
      PaintRegion(ho_Edges, image, &ho_addedImage, ((HTuple(0).Append(255)).Append(0)), "fill");

      hv_save_filename = (root + HalconCpp::HTuple(index)) + "_added.jpg";
      WriteImage(ho_addedImage, "jpeg 100", 0, hv_save_filename);
    } catch (HOperatorException e) {
      std::string errorMessage = e.ErrorMessage().Text();
      ROS_ERROR_STREAM(errorMessage);
    }
  }
  ROS_INFO_STREAM("Finished adding caltabs.");
}

void Detector::findCaltabAction(const calibration_executive::FindCalibrationObjectGoalConstPtr &goal) {
  ROS_INFO_STREAM("Received FindCaltabAction");
  callbacksLeft = goal->retries;
  caltabFound = false;
  ros::Time timeToStop = ros::Time::now() + ros::Duration(goal->timeout);

  while (callbacksLeft > 0 && !caltabFound && timeToStop - ros::Time::now() > ros::Duration(0)) {
    rate.sleep();
  }
  if (caltabFound) {
    ROS_INFO_STREAM("FindCaltabAction succeeded");
    findCaltabResult.result = 1;
    findCaltabResult.position = caltabPosition;
    findCaltabResult.orientation = caltabOrientation;
    findCaltabActionServer.setSucceeded(findCaltabResult);
  } else {
    ROS_INFO_STREAM("FindCaltabAction failed");
    findCaltabResult.result = 0;
    findCaltabActionServer.setSucceeded(findCaltabResult);
  }
}

void Detector::calibrateAction(const calibration_executive::CalculateParametersGoalConstPtr &goal) {
  ROS_INFO_STREAM("Received CalibrateAction");

  HTuple error, newCameraParam, result;
  try {
    //Actually do the calibration algorithm
    CalibrateCameras(calibDataId, &error);

    //Extract some results from the calibration
    GetCalibData(calibDataId, "camera", 0, "params", &newCameraParam);
    //Param
    TupleLastN(newCameraParam, 1, &result);

    std::vector<double> resultsListDouble;
    for (int i = 0; i < result.Length(); i++) {
      resultsListDouble.push_back(result[i].D());
    }

    calculateParametersResult.number_of_images = suitablePictureIDs.Length();
    calculateParametersResult.intrinsics = resultsListDouble;
    calculateParametersResult.error = error;

    calibrateActionServer.setSucceeded(calculateParametersResult);
  } catch (HOperatorException e) {
    std::string errorMessage = e.ErrorMessage().Text();
    ROS_ERROR_STREAM(errorMessage);
  }

  addCaltabToImages();

  ros::shutdown();
}

void Detector::createInitialParameters(double focal_length, double sensor_size_x, double sensor_size_y,
                                       int img_resolution_x, int img_resolution_y, HTuple &cameraParams) {
  HTuple hv_camera_type, hv_s_Kappa, pixel_size_x, pixel_size_y;
  hv_camera_type = "area_scan_polynomial";
  hv_s_Kappa.Clear();
  hv_s_Kappa[0] = 0.0;
  hv_s_Kappa[1] = 0.0;
  hv_s_Kappa[2] = 0.0;
  hv_s_Kappa[3] = 0.0;
  hv_s_Kappa[4] = 0.0;

  pixel_size_x = sensor_size_x / (float) img_resolution_x;
  pixel_size_y = sensor_size_y / (float) img_resolution_y;

  cameraParams.Clear();
  cameraParams.Append(hv_camera_type);
  cameraParams.Append(focal_length);
  cameraParams.Append(hv_s_Kappa);
  cameraParams.Append(pixel_size_x);
  cameraParams.Append(pixel_size_y);
  cameraParams.Append(img_resolution_x / 2);
  cameraParams.Append(img_resolution_y / 2);
  cameraParams.Append(img_resolution_x);
  cameraParams.Append(img_resolution_y);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "calibration_perception_node");
  Detector det;
  det.init();
  ros::spin();
}
