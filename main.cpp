#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common.h>

#include <opengv/relative_pose/methods.hpp>
#include <opengv/relative_pose/CentralRelativeAdapter.hpp>

#include <camera.h>

using namespace cv;
using namespace cv::xfeatures2d;


void main()
{
  Camera cam;
  cam.read("E:/visual/calib.yml");

  //first image
  cv::Mat img_1 = cv::imread("E:/visual/test_1/IMG_8588.jpg");

  double scale = 0.125;
  
  
  cv::resize(img_1, img_1, Size(), scale, scale, INTER_LINEAR);

  cam.resizeCamera(scale);

  cv::imshow("image1", img_1);

  cv::Ptr<cv::xfeatures2d::SURF> detector = cv::xfeatures2d::SURF::create(400);
  std::vector<cv::KeyPoint> keypoints1;
  Mat descriptors1;
  detector->detectAndCompute(img_1, noArray(), keypoints1, descriptors1);

  //todo undistort keyPoints

  for(const auto& p : keypoints1)
  {
    cv::circle(img_1, p.pt, 3, Scalar(255,0,255), 1);
  }

  auto bearings1 = cam.getBearings(keypoints1);

  cv::imshow("key points1", img_1);

  pcl::console::setVerbosityLevel(pcl::console::VERBOSITY_LEVEL::L_ALWAYS);

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
  viewer->setBackgroundColor(0, 0, 0);
  viewer->addCoordinateSystem(1.0);
  viewer->initCameraParameters();

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());


  for (const auto& b : bearings1)
  {
    pcl::PointXYZ p(b.x(), b.y(), b.z());
    cloud->points.push_back(p);
  }
  
  viewer->addPointCloud<pcl::PointXYZ>(cloud);

  //second image
  cv::Mat img_2 = cv::imread("E:/visual/test_1/IMG_8589.jpg");
  cv::resize(img_2, img_2, Size(), 0.125,0.125, INTER_LINEAR);
  cv::imshow("image2", img_2);

  std::vector<cv::KeyPoint> keypoints2;
  cv::Mat descriptors2;
  detector->detectAndCompute(img_2, noArray(), keypoints2, descriptors2);

  //todo undistort keyPoints

  for(const auto& p: keypoints2)
  {
    cv::circle(img_2, p.pt, 3, Scalar(255, 0, 255) , 1);
  }

  auto bearings2 = cam.getBearings(keypoints2);

  cv::imshow("key points2", img_2);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2(new pcl::PointCloud<pcl::PointXYZ>());
  for(const auto& p : bearings2)
  {
    pcl::PointXYZ tmp(p.x(), p.y(), p.z());
    cloud_2->points.push_back(tmp);
  }

  viewer->addPointCloud<pcl::PointXYZ>(cloud_2,"cloud_2");
  viewer->setPointCloudRenderingProperties(4, 0.1, 0.2, 0.3, "cloud_2");

  while (!viewer->wasStopped())
  {
    viewer->spinOnce(50, true);
  }


  //matching points
  cv::Ptr<DescriptorMatcher> matcher = cv::DescriptorMatcher::create(DescriptorMatcher::FLANNBASED);
  

  std::vector<std::vector<cv::DMatch>> matches;

  matcher->knnMatch(descriptors1, descriptors2, matches, 2);
  

  cv::Mat result;
  cv::drawMatches(img_1, keypoints1,img_2,keypoints2, matches, result);
  cv::imshow("matches", result);
  //cv::waitKey(-1);


  std::vector<cv::DMatch> good_matches;
  
  double ratio_threshold = 0.7;

  for(const auto& m : matches)
  {
    if(m[0].distance < ratio_threshold*m[1].distance)
    {
      good_matches.push_back(m[0]);
    }
  }

  std::vector<int> indices(matches.size());

  for(auto i = 0; i < matches.size(); i++)
  {
    indices[i] = matches[i][0].trainIdx;
  }


  cv::Mat good_result;
  cv::drawMatches(img_1, keypoints1, img_2, keypoints2, good_matches, good_result);
  cv::imshow("good_matches", good_result);
  cv::waitKey(-1);

  ///
  opengv::relative_pose::CentralRelativeAdapter adapter(bearings1, bearings2);

  opengv::essentials_t sevenpt_essentials = opengv::relative_pose::sevenpt(adapter);

  for(const auto& t : sevenpt_essentials)
  {
    std::cout << t << std::endl;
  }
  
}