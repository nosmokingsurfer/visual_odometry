#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <camera.h>
#include <odometer.h>
#include <helpers.h>
#include <iostream>
#include <fstream>


void main(int argc, char** argv)
{
  std::string calibDir = std::string(argv[0]) + "/../../../";
  std::string datasetDir = std::string(argv[0]) + "/../../../";

  //Odometer<cv::ORB, cv::DescriptorMatcher::FLANNBASED> odo(0.125);
  Odometer<cv::xfeatures2d::SURF, cv::DescriptorMatcher::BRUTEFORCE> odo(0.125);
  odo.init();
  odo.initCam(calibDir + "calib.yml");
  
  std::ofstream output;
  output.open("out.txt");
  if (output.is_open())
  {
    output.clear();
  }
  output.close();


  std::vector<std::string> files = load_test(datasetDir + "/test_3");
  
  for(const auto& f : files)
  {
    std::cout << f <<  std::endl;
    cv::Mat img = cv::imread(f);
    odo.process(img);
  }
  
}