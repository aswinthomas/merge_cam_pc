#ifndef MY_POINTCLOUD_H_
#define MY_POINTCLOUD_H_

// pcl
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// opencv
#include <opencv2/opencv.hpp>

struct MyPoint {
  PCL_ADD_POINT4D;
  float intensity;
  uint16_t layer;
  float range;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

class MyPointCloud {
public:
  MyPointCloud(::pcl::PointCloud<MyPoint> pc);
  MyPointCloud transform(std::vector<float> cal);
  MyPointCloud transform(float x, float y, float z, float rot_x, float rot_y,
                         float rot_z);
  static cv::Point2f projectf(const MyPoint &pt,
                              const cv::Mat &projection_matrix) {
    cv::Mat pt_3D(4, 1, CV_32FC1);

    pt_3D.at<float>(0) = pt.x;
    pt_3D.at<float>(1) = pt.y;
    pt_3D.at<float>(2) = pt.z;
    pt_3D.at<float>(3) = 1.0f;

    cv::Mat pt_2D = projection_matrix * pt_3D;

    float w = pt_2D.at<float>(2);
    float x = pt_2D.at<float>(0) / w;
    float y = pt_2D.at<float>(1) / w;

    return cv::Point2f(x, y);
  }

  static cv::Point project(const MyPoint &pt,
                           const cv::Mat &projection_matrix) {
    cv::Point2f xy = projectf(pt, projection_matrix);
    return cv::Point(xy.x, xy.y);
  }
  cv::Mat project(cv::Mat projection_matrix, cv::Rect frame,
                  ::pcl::PointCloud<MyPoint> *visible_points = NULL);
  cv::Mat project(cv::Mat projection_matrix, cv::Rect frame, cv::Mat plane);

  ::pcl::PointCloud<pcl::PointXYZRGBA> segmentedColor(cv::Mat frame_rgb,
                                                      cv::Mat P);

protected:
  ::pcl::PointCloud<MyPoint> pc_;
};

POINT_CLOUD_REGISTER_POINT_STRUCT(
    MyPoint,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity,
                                            intensity)(uint16_t, layer, layer))

#endif
