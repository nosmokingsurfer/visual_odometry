#include <camera.h>

using namespace Eigen;

Vector3f Camera::reprojectPtWithDist(Vector2i pixel, float meterDist) const
{
  float ax = (pixel[0] - projection_(0, 2)) / projection_(0, 0);
  float ay = (pixel[1] - projection_(1, 2)) / projection_(1, 1);
  Vector3f dirVec;
  dirVec << ax, ay, 1;

  float z = meterDist / dirVec.norm();
  Vector3f preres = dirVec * z;
  return preres;
}

Camera::Camera()
  : pose_(Pose())
{
  

  //some camera parameters
  this->distortion_ << -1.1983283309789111e-01, 2.7076763925130121e-01, 0., 0., -7.3458604303021896e-02;

  this->projection_ << 140, 0., 70,
                       0., 140, 70,
                       0., 0., 1.;

  this->imageSize_ << 140, 140;
}



Camera::Camera(const Pose & pose_)
{
  (*this) = Camera();
  this->pose_ = pose_;
}

Camera::Camera(const Matrix3f &K, const Matrix<float,5,1> &D, const Pose& pose, const Vector2i &sz)
  : projection_(K)
  , distortion_(D)
  , imageSize_(sz)
  , pose_(pose)
{
}

Vector2i Camera::projectPt(Vector3f pt) const
{
  Vector3f result = projection_ * pt;
  if (result[2])
    return Vector2i(result[0] / result[2], result[1] / result[2]);
  return Vector2i();
}