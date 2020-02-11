#include <camera.h>
#include <iostream>
#include <opencv2/imgproc.hpp>

Camera::Camera()
{
   
}

bool Camera::read(const std::string& fileName)
{
  cv::FileStorage fs;
  fs.open(fileName, cv::FileStorage::Mode::READ);

  if(fs.isOpened())
  {
    fs["K"] >> this->projection;
    fs["D"] >> this->distortion;
    fs["imageSize"] >> this->imgSize;
  }

  return true;
}

bool Camera::resizeCamera(const double scale)
{
  
  this->imgSize = cv::Size(imgSize.width*scale, imgSize.height*scale);

  for(auto row = 0; row < projection.rows; row++)
  {
    for(auto col = 0; col < projection.row(row).cols; col++)
    {
      projection.row(row).col(col) *= scale;
    }
  }

  return true;
}

Eigen::Vector3d Camera::reprojectWithDist(const cv::Point& pt, const double& dist) const
{
  Eigen::Vector3d result;

    result[0]= (pt.x - projection.at<double>(0, 2)) / projection.at<double>(0, 0);
    result[1] = (pt.y - projection.at<double>(1, 2)) / projection.at<double>(1, 1);
    result[2] = 1.0;

    result *= dist/result.norm();

  return result;
}


opengv::bearingVectors_t Camera::getBearings(const std::vector<cv::Point2f>& pts, const double dist) const
{
  opengv::bearingVectors_t result(pts.size());

  for(const auto&pt : pts)
  {
    result.push_back(reprojectWithDist(pt, dist));
  }

  return result;
}

opengv::bearingVectors_t Camera::getBearings(const std::vector<cv::KeyPoint>& kPts, const double dist) const
{
  opengv::bearingVectors_t result(kPts.size());

  for(const auto& kpt : kPts)
  {
    result.push_back(reprojectWithDist(kpt.pt, dist));
  }

  return result;
}

cv::Size Camera::getImageSize(void) const
{
  return this->imgSize;
}

const cv::Mat Camera::getProjection() const
{
  return this->projection;
}

const cv::Mat Camera::getDistortion() const
{
  return this->distortion;
}

bool Camera::setCamParams(const cv::Mat& projection_, const cv::Mat& distortion_, const cv::Size& sz_)
{
  this->projection = projection_;
  this->distortion = distortion_;
  this->imgSize = sz_;

  return true;
}

Camera Camera::getUndistortedCamera() const
{
  Camera result;

  cv::Mat src(this->imgSize, CV_32F);
  cv::Mat dst(this->imgSize, CV_32F);

  cv::Mat new_projection = projection;
  cv::Mat new_distortion = distortion;

  cv::undistort(src, dst, projection, distortion, new_projection);

  new_distortion = distortion;
  new_distortion.setTo(0);

  result.setCamParams(new_projection, new_distortion, this->imgSize);
  
  return result;
}