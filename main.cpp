#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common.h>

#include <camera.h>

#include <odometer.h>

#include <helpers.h>

using namespace cv;
using namespace cv::xfeatures2d;


void main()
{

  Odometer odo(Odometer::KEYPOINTS::SURF, Odometer::MATCHERS::FLANNBASED, 0.125);
  odo.init();

  odo.initCam("E:/visual/calib.yml");

  std::vector<std::string> files = load_test_2();
  
  
  for(const auto& f : files)
  {
    cv::Mat img = cv::imread(f);
    odo.process(img);
  }
  
}