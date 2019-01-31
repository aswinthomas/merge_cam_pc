#ifndef MY_POINTCLOUD_H_
#define MY_POINTCLOUD_H_

// pcl
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// opencv
#include <opencv2/opencv.hpp>

struct MyPoint {
  PCL_ADD_POINT4D
  float range;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW // ensure proper alignment
} EIGEN_ALIGN16;

class MyPointCloud {
public:
  MyPointCloud(::pcl::PointCloud<MyPoint> pc);
  MyPointCloud transform(std::vector<float> cal);
  MyPointCloud transform(float x, float y, float z, float rot_x,
                                     float rot_y, float rot_z);
  cv::Point2f projectf(const MyPoint &pt, const cv::Mat &projection_matrix);
  cv::Point project(const MyPoint &pt, const cv::Mat &projection_matrix);
  cv::Mat project(cv::Mat projection_matrix, cv::Rect frame,
                  ::pcl::PointCloud<MyPoint> *visible_points = NULL);
  cv::Mat project(cv::Mat projection_matrix, cv::Rect frame, cv::Mat plane);

  ::pcl::PointCloud<pcl::PointXYZRGBA> segmentedColor(cv::Mat frame_rgb,
                                                      cv::Mat P);

protected:
  ::pcl::PointCloud<MyPoint> pc_;
};

#endif
