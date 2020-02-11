//! @file odometer.h
//! @brief Odometer class to calculate all the odometry
//! @author Panchenko A.V.
//! @date 2019.12
//! 

#pragma once
#ifndef ODOMETER_H__2019
#define ODOMETER_H__2019


#include <camera.h>
#include <opengv/types.hpp>
#include <opencv2/xfeatures2d.hpp>


//! @brief Class to work with visal and IMU odometry.
class Odometer
{
  public:
    enum class KEYPOINTS
    {
      SURF,
      SIFT,
      ORB
    };

    enum class MATCHERS
    {
      FLANNBASED,
      BRUTEFORCE,
      BRUTEFORCE_L1
    };

    Odometer(const KEYPOINTS& keyPointType, const MATCHERS& matcherType, const double scale);

    bool init();

    //! @brief Processes new image.
    //! @retval bool - True - if success, false - if fail.
    bool process(const cv::Mat& newImg);

    //@ brief Calculates bearing vectors in terms of OpenGV. Takes into account radial distortion of the camera
    opengv::bearingVectors_t extractBearings(const std::vector<cv::KeyPoint>& keyPoints);

    bool initCam(const std::string& calibFile);
  private:
    Camera cam_; //!< Store camera properies and model.
    cv::Mat prevImage_; //!< Prevous image.
    std::vector<cv::KeyPoint> prevKeyPoints_; //!< Previous set of key points.
    cv::Mat prevDescriptors_; //!< Previous set of descriptors.
    opengv::bearingVectors_t prevBearingVectors_; //!M Previous bearing vectors.
    KEYPOINTS keyPointsType_; //!< Type of key points used in visual odometry.
    MATCHERS matcherType_; //!< Type of key point matcher.
    double imageScale_; //!< Image scale for processing.
    bool initialized = false; //!< If Odometer was initialized or not.

    //key point detectors
    cv::Ptr<cv::xfeatures2d::SURF> surfDetector_; //!< Instance for SURF detector.

    //key point matchers
    cv::Ptr<cv::DescriptorMatcher> matcher_; //!< Instance for key point matcher.
};

#endif //ODOMETER_H__2019