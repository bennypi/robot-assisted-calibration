/*
 * example_creation.cpp
 *
 *  Created on: May 30, 2017
 *      Author: benny
 */

#include "HalconCpp.h"

#include "ros/ros.h"

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>


int getCaltabPose();
int createCameraParams();

int main() {
  //return getCaltabPose();
  return createCameraParams();

}

int createCameraParams() {
  try {
    HalconCpp::HTuple hv_start_cam_par, hv_camera_type, hv_s_Focus, hv_s_Kappa, hv_s_Sx, hv_s_Sy, hv_img_res_x, hv_img_res_y, hv_sensor_size_x, hv_sensor_size_y, Path;

    hv_img_res_x = 1920;
    hv_img_res_y = 1080;

    //3.7mm lens
    hv_s_Focus = 0.0037;

    //s_Kappa := 0.0
    //camera_type := 'area_scan_division'
    hv_camera_type = "area_scan_polynomial";
    hv_s_Kappa.Clear();
    hv_s_Kappa[0] = 0.0;
    hv_s_Kappa[1] = 0.0;
    hv_s_Kappa[2] = 0.0;
    hv_s_Kappa[3] = 0.0;
    hv_s_Kappa[4] = 0.0;

    hv_sensor_size_x = 0.00585;
    hv_sensor_size_y = 0.00327;
    hv_s_Sx = hv_sensor_size_x / hv_img_res_x;
    hv_s_Sy = hv_sensor_size_y / hv_img_res_y;

    hv_start_cam_par.Clear();
    hv_start_cam_par.Append(hv_camera_type);
    hv_start_cam_par.Append(hv_s_Focus);
    hv_start_cam_par.Append(hv_s_Kappa);
    hv_start_cam_par.Append(hv_s_Sx);
    hv_start_cam_par.Append(hv_s_Sy);
    hv_start_cam_par.Append(hv_img_res_x / 2);
    hv_start_cam_par.Append(hv_img_res_y / 2);
    hv_start_cam_par.Append(hv_img_res_x);
    hv_start_cam_par.Append(hv_img_res_y);

    Path.Append("/home/bennypi/kinect2_hd_inital_calib/microsoft_camera.dat");

    HalconCpp::WriteCamPar(hv_start_cam_par, Path);

  }
  catch (HalconCpp::HException &HDevExpDefaultException) {
    std::cout << "exception while creating camera params" << std::endl;
    std::cout << HDevExpDefaultException.ErrorMessage() << std::endl;

  }
  return 0;
}

  int getCaltabPose() {
    HalconCpp::HTuple file, calibDataId, cameraParameters, calibObjDescr, hv_CameraParam, hv_NFinalPose;
    try {
      const char *initalParameters = "/home/bennypi/kinect2_hd_inital_calib/camera.dat";
      file.Append("/home/bennypi/kinect2_hd_inital_calib/inspect_images/success/15.jpg");
      CreateCalibData("calibration_object", 1, 1, &calibDataId);
      calibObjDescr.Append("/home/bennypi/kinect2_hd_inital_calib/caltab_hex_10x11.cpd");
      SetCalibDataCalibObject(calibDataId, 0, calibObjDescr);
      ReadCamPar(initalParameters, &cameraParameters);
      SetCalibDataCamParam(calibDataId, 0, "", cameraParameters);


    } catch (const HalconCpp::HOperatorException e) {
      std::cout << "exception during initialization" << std::endl;
      std::cout << e.ErrorMessage() << std::endl;
      return 0;
    }

    try {
      //Reading the image by using the standart image description *
      HalconCpp::HObject ho_Image;
      HalconCpp::HTuple hv_Error;
      ReadImage(&ho_Image, file);


      //find calibration pattern
      //Test both to know if the caltab was found and if the different circles were too *
      FindCalibObject(ho_Image, calibDataId, 0, 0, 0, HalconCpp::HTuple(), HalconCpp::HTuple());
      //find_calib_object (Image, CalibDataID, 0, 0, i, ['gap_tolerance', 'alpha', 'max_diam_marks', 'skip_find_caltab'], [0.9, 0.5, 10, 'false'])

      std::cout << "success" << std::endl;
      // catch (Exception)
      CalibrateCameras(calibDataId, &hv_Error);
      std::cout << "success" << std::endl;
      GetCalibData(calibDataId, "camera", 0, "params", &hv_CameraParam);
      GetCalibData(calibDataId, "camera", 0, "pose", &hv_NFinalPose);
      std::cout << hv_NFinalPose.ToString() << std::endl;
      std::cout << hv_CameraParam.ToString() << std::endl;
      std::cout << hv_Error.ToString() << std::endl;

    }
    catch (HalconCpp::HException &HDevExpDefaultException) {
      std::cout << "exception during reading" << std::endl;
      std::cout << HDevExpDefaultException.ErrorMessage() << std::endl;
      return 0;
    }


    try {
      HalconCpp::HTuple hv_Pose, newFile, newPose, points, row, col, idx;
      HalconCpp::HObject ho_Image, ho_simImage, ho_simAmp, ho_simDir, ho_Edges, ho_addedImage, multiChannel;
      ReadImage(&ho_Image, file);
      ChannelsToImage(ho_Image, &multiChannel);
      //load caltab pose from the CalibDataID object
      GetCalibData(calibDataId, "calib_obj_pose", HalconCpp::HTuple(0).TupleConcat(0),
                   "pose", &hv_Pose);
      std::cout << "success" << std::endl;
      std::cout << hv_Pose.ToString() << std::endl;
      GetCalibDataObservPose(calibDataId, 0, 0, 0, &newPose);
      std::cout << "success" << std::endl;
      std::cout << newPose.ToString() << std::endl;
      GetCalibDataObservPoints(calibDataId, 0, 0, 0, &row, &col, &idx, &points);
      std::cout << "success" << std::endl;
      std::cout << row.ToString() << std::endl;
      std::cout << col.ToString() << std::endl;
      std::cout << idx.ToString() << std::endl;
      std::cout << points.ToString() << std::endl;
      SimCaltab(&ho_simImage, calibObjDescr, hv_CameraParam, hv_Pose, 0, 80,
                224, 1);
      EdgesImage(ho_simImage, &ho_simAmp, &ho_simDir, "canny", 1.0, "nms", 20,
                 40);
      Threshold(ho_simAmp, &ho_Edges, 1, 255);
      //skeleton (Edges, Skeleton)
      //gen_contours_skeleton_xld (Skeleton, Contours, 1, 'filter')


      //The following two lines make a grayish painting of the caltab on top of the image
      //compose3 (simAmp, simAmp, simAmp, simImage3)
      //add_image (Image, simImage3, addedImage, 0.5, 0)

      //This one paints only the edges
      PaintRegion(ho_Edges, multiChannel, &ho_addedImage, ((HalconCpp::HTuple(0).Append(255)).Append(0)),
                  "fill");

      newFile.Append("/home/bennypi/kinect2_hd_inital_calib/inspect_images/success/15_added.jpg");
      WriteImage(ho_addedImage, "jpeg 100", 0, newFile);
      //save_filename := pathImageDir + '/projection/res_frame' + i + '.png'
      //write_image (addedImage, 'png fastest', 0, save_filename)

    }
    catch (HalconCpp::HException &HDevExpDefaultException) {
      std::cout << "exception during extra stuff" << std::endl;
      std::cout << HDevExpDefaultException.ErrorMessage() << std::endl;
      return 0;
    }
    return 0;
  }


