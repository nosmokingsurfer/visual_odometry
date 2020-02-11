///
/// @file camera.h
///@authors Panchenko A.V.
///@brief Camera class - OOP class to work with camera mathematical model
///
///

#pragma once
#ifndef CAMERA_H
#define CAMERA_H

#include <Eigen/Dense>

#include <opencv2/core.hpp>

#include <opengv/types.hpp>

using namespace Eigen;

class Camera
{
  public:
    Camera();

    bool read(const std::string& fileName);

    bool Camera::resizeCamera(const double scale);

    Eigen::Vector3d reprojectWithDist(const cv::Point& pt, const double& dist) const;

    cv::Size Camera::getImageSize(void) const;

    opengv::bearingVectors_t Camera::getBearings(const std::vector<cv::KeyPoint>& kPts, const double dist = 1.0) const;
    opengv::bearingVectors_t Camera::getBearings(const std::vector<cv::Point2f>& pts, const double dist = 1.0) const;

    const cv::Mat getProjection(void) const;
    const cv::Mat getDistortion(void) const;

    Camera getUndistortedCamera() const;

    bool setCamParams(const cv::Mat& projection, const cv::Mat& distortion, const cv::Size& sz);

  private:
    cv::Size imgSize;
    cv::Mat projection;
    cv::Mat distortion;
};

#endif //CAMERA_H