#include <odometer.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/imgproc.hpp>

#include <opengv/relative_pose/methods.hpp>
#include <opengv/relative_pose/CentralRelativeAdapter.hpp>

#include <iostream>


Odometer::Odometer(const Odometer::KEYPOINTS& keyPointType, const MATCHERS& matcherType, const double scale)
: keyPointsType_(keyPointType)
, imageScale_(scale)
, matcherType_(matcherType)
{};

 
 opengv::bearingVectors_t Odometer::extractBearings(const std::vector<cv::KeyPoint>& keyPoints)
{
  //undistorting keyPoints
  std::vector<cv::Point2f> src(keyPoints.size());
  for(auto i = 0; i < keyPoints.size(); i++)
  {
    src[i] = keyPoints[i].pt;
  }


  std::vector<cv::Point2f> dst(keyPoints.size());
  cv::undistortPoints(src, dst, cam_.getProjection(), cam_.getDistortion());
  
  //calculating bearing for undistorted keypoints
  Camera undistorted_cam = cam_.getUndistortedCamera(); //get undistorted camera
  auto bearings = undistorted_cam.getBearings(dst);

  //returning bearings
  return bearings;
}

 bool Odometer::initCam(const std::string& calibFile)
 {
  bool res = this->cam_.read(calibFile);
  res &= cam_.resizeCamera(this->imageScale_);
  return res;
 }


 bool Odometer::init()
 {
  switch(keyPointsType_)
  {
    case(KEYPOINTS::SURF):
    {
      surfDetector_ = cv::xfeatures2d::SURF::create(100);
      break;
    }

    case(KEYPOINTS::SIFT):
    {
      break;
    }
  }

  switch(matcherType_)
  {
    case(MATCHERS::FLANNBASED):
    {
      matcher_ = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
      break;
    }
  }

  initialized = true;
  return true;
 }


 bool Odometer::process(const cv::Mat& newImg)
 {
  //processing new image
  cv::Mat resized_img;
  cv::resize(newImg, resized_img, cv::Size(), imageScale_, imageScale_, cv::INTER_LINEAR);
  cv::imshow("image_1", resized_img);

  std::vector<cv::KeyPoint> newKeyPoints;
  cv::Mat newDescriptors;

  switch (this->keyPointsType_)
  {
    case(KEYPOINTS::SURF):
    {
      this->surfDetector_->detectAndCompute(resized_img, cv::noArray(), newKeyPoints, newDescriptors);
      break;
    }
  };

  auto newBearings = extractBearings(newKeyPoints);

  cv::drawKeypoints(resized_img, newKeyPoints, resized_img);
  cv::imshow("new keypoints", resized_img);

  //doing matching with previous image
  std::vector<int> indices;
  if(prevKeyPoints_.size() != 0)
  {
    std::vector<cv::DMatch> goodMatches;
    double ratio_threshold = 0.7;
    std::vector<std::vector<cv::DMatch>> matches;
    matcher_->knnMatch(prevDescriptors_, newDescriptors, matches, 2);

    for(const auto& m : matches)
    {
      if(m[0].distance < ratio_threshold*m[1].distance)
      {
        goodMatches.push_back(m[0]);
      }
    }


    indices.resize(goodMatches.size());
    for(auto i = 0; i < goodMatches.size(); i++)
    {
      indices[i] = goodMatches[i].trainIdx;
    }
  }
  else
  {
    prevDescriptors_ = newDescriptors;
    prevImage_ = newImg;
    prevKeyPoints_ = newKeyPoints;
    prevBearingVectors_ = newBearings;
    return false;
  }

  //estimating transition and rotation

  opengv::relative_pose::CentralRelativeAdapter adapter(prevBearingVectors_, newBearings);  
  auto result = opengv::relative_pose::optimize_nonlinear(adapter);

  std::cout << result << std::endl;


  prevDescriptors_ = newDescriptors;
  prevImage_ = newImg;
  prevKeyPoints_ = newKeyPoints;
  prevBearingVectors_ = newBearings;

  cv::waitKey(50);
  return true;
 }
