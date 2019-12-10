#include <camera.h>


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


opengv::bearingVectors_t Camera::getBearings(const std::vector<cv::KeyPoint>& kPts, const double& dist) const
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