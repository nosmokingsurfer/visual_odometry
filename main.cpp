#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <camera.h>
#include <odometer.h>
#include <helpers.h>
#include <iostream>


void main(int argc, char** argv)
{

  std::cout << argv[0] << std::endl;

  std::string calibDir = std::string(argv[0]) + "/../../../";
  std::string datasetDir = std::string(argv[0]) + "/../../../";

  Odometer<cv::ORB, cv::DescriptorMatcher::FLANNBASED> odo(0.125);
  odo.init();
  odo.initCam(calibDir + "calib.yml");
  

  std::vector<std::string> files = load_test(datasetDir + "/test_2");
  
  for(const auto& f : files)
  {
    cv::Mat img = cv::imread(f);
    odo.process(img);
  }
  
}