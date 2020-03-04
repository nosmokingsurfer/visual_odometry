#include <odometer.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/eigen.hpp>

#include <opengv/relative_pose/methods.hpp>
#include <opengv/relative_pose/CentralRelativeAdapter.hpp>

#include <iostream>
#include <fstream>


template<typename keypoint, cv::DescriptorMatcher::MatcherType type>
Odometer<keypoint, type>::Odometer(const double scale):
  imageScale_(scale)
{
  this->keypointDetector_ = keypoint::create();

  this->matcher_ = cv::DescriptorMatcher::create(type);
}

template<typename keypoint, cv::DescriptorMatcher::MatcherType matcher>
 opengv::bearingVectors_t Odometer<keypoint, matcher>::extractBearings(const std::vector<cv::KeyPoint>& keyPoints)
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

 template<typename keypoint, cv::DescriptorMatcher::MatcherType matcher>
 bool Odometer<keypoint, matcher>::initCam(const std::string& calibFile)
 {
  bool res = this->cam_.read(calibFile);
  res &= cam_.resizeCamera(this->imageScale_);
  return res;
 }

 template<typename keypoint, cv::DescriptorMatcher::MatcherType matcher>
 bool Odometer<keypoint, matcher>::init()
 {

  initialized = true;
  return true;
 }

 template<typename keypoint, cv::DescriptorMatcher::MatcherType matcher>
 bool Odometer<keypoint, matcher>::process(const cv::Mat& newImg)
 {
  //processing new image
  cv::Mat resized_img;
  cv::resize(newImg, resized_img, cv::Size(), imageScale_, imageScale_, cv::INTER_LINEAR);
  cv::imshow("image_1", resized_img);

  std::vector<cv::KeyPoint> newKeyPoints;
  cv::Mat newDescriptors;
  
  this->keypointDetector_->detectAndCompute(resized_img, cv::noArray(), newKeyPoints, newDescriptors);

  newDescriptors.convertTo(newDescriptors, CV_32F);

  auto newBearings = extractBearings(newKeyPoints);

  cv::drawKeypoints(resized_img, newKeyPoints, resized_img);// , cv::Scalar(), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
  cv::imshow("new keypoints", resized_img);

  
  if (!prevImage_.empty())
  {
    cv::drawKeypoints(prevImage_, prevKeyPoints_, prevImage_);// , cv::Scalar(), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    cv::imshow("prev key points", prevImage_);
  }

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
      else if (m[1].distance < ratio_threshold * m[0].distance)
      {
        goodMatches.push_back(m[1]);
      }
    }

    cv::Mat matchesImg;
    cv::drawMatches(prevImage_, prevKeyPoints_, resized_img, newKeyPoints, goodMatches, matchesImg);
    cv::imshow("Matches", matchesImg);


    indices.resize(goodMatches.size());
    for(auto i = 0; i < goodMatches.size(); i++)
    {
      indices[i] = goodMatches[i].imgIdx;
    }
  }
  else
  {
    prevDescriptors_ = newDescriptors;
    cv::resize(newImg, prevImage_, cv::Size(), imageScale_, imageScale_, cv::INTER_LINEAR);
    prevKeyPoints_ = newKeyPoints;
    prevBearingVectors_ = newBearings;
    return false;
  }

  //estimating transition and rotation
  opengv::relative_pose::CentralRelativeAdapter adapter(prevBearingVectors_, newBearings);  
  opengv::essential_t result = opengv::relative_pose::eightpt(adapter, indices);
  
  //opengv::transformation_t result = opengv::relative_pose::optimize_nonlinear(adapter, indices);

  std::cout << result << std::endl;

  cv::Mat R1;
  cv::Mat R2;
  cv::Mat t;
 
  cv::Mat E;
  cv::eigen2cv(result, E);

  cv::decomposeEssentialMat(E, R1, R2, t);

  std::cout << "R1 = " << R1 << std::endl;
  std::cout << "R2 = " << R2 << std::endl;
  std::cout << "t = " << t << std::endl;

  std::ofstream output;
  output.open("out.txt", std::ios_base::app);

  cv::Vec3d t_vec(t);

  if (output.is_open())
  {
    output << t_vec[0] << " " << t_vec[1] << " " << t_vec[2] << std::endl;
  }

  output.close();

  prevDescriptors_ = newDescriptors;
  cv::resize(newImg, prevImage_, cv::Size(), imageScale_, imageScale_, cv::INTER_LINEAR);
  
  prevKeyPoints_ = newKeyPoints;
  prevBearingVectors_ = newBearings;

  int key = cv::waitKey(50);
  if (key == 32) // spacebar pressed
  {
    bool stopped = true;
    key = -1;
    while (stopped)
    {
      key = cv::waitKey(-1);
      if (key == 32)
      {
        stopped = false;
      }
      else if (key == 27)
      {
        exit(1);
      }
    }
  }
  return true;
 }

 template Odometer<cv::ORB, cv::DescriptorMatcher::FLANNBASED>;
 template Odometer<cv::xfeatures2d::SIFT, cv::DescriptorMatcher::BRUTEFORCE>;
 template Odometer<cv::xfeatures2d::SIFT, cv::DescriptorMatcher::FLANNBASED>;
 template Odometer<cv::xfeatures2d::SURF, cv::DescriptorMatcher::BRUTEFORCE>;