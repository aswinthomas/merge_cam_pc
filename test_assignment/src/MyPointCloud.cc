#include <cmath>
#include <vector>

#include <test_assignment/MyPointCloud.h>

#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <ros/assert.h>
#include <tf/transform_listener.h>

using namespace std;
using namespace cv;
using namespace pcl;
using namespace ros;

MyPointCloud::MyPointCloud(PointCloud<MyPoint> pc) : pc_(pc) {}

MyPointCloud MyPointCloud::transform(float x, float y, float z, float rot_x,
                                     float rot_y, float rot_z) {
  Eigen::Affine3f transf = getTransformation(x, y, z, rot_x, rot_y, rot_z);
  PointCloud<Point> new_cloud;
  transformPointCloud(point_cloud, new_cloud, transf);
  return MyPointCloud(new_cloud);
}

MyPointCloud MyPointCloud::transform(vector<float> cal) {
  ROS_ASSERT(cal.size() == 6);
  return transform(cal[0], cal[1], cal[2], cal[3], cal[4], cal[5]);
}

cv::Point2f projectf(const Point &pt, const cv::Mat &projection_matrix) {
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

cv::Point project(const Point &pt, const cv::Mat &projection_matrix) {
  cv::Point2f xy = projectf(pt, projection_matrix);
  return cv::Point(xy.x, xy.y);
}

cv::Mat MyPointCloud::project(Mat projection_matrix, Rect frame,
                              PointCloud<MyPoint> *visible_points) {
  Mat plane = cv::Mat::zeros(frame.size(), CV_32FC1);

  for (PointCloud<Point>::iterator pt = point_cloud.points.begin();
       pt < point_cloud.points.end(); pt++) {

    // ignore if behind camera
    if (pt->z < 0)
      continue;
    cv::Point xy = project(*pt, projection_matrix);
    if (xy.inside(frame)) {
      if (visible_points != NULL) {
        visible_points->push_back(*pt);
      }
    }
  }

  Mat plane_gray;
  cv::normalize(plane, plane_gray, 0, 255, NORM_MINMAX, CV_8UC1);
  dilate(plane_gray, plane_gray, Mat());

  return plane_gray;
}

Mat Velodyne::Velodyne::project(Mat projection_matrix, Rect frame, Mat image) {
  Mat plane = project(projection_matrix, frame, NULL);

  ROS_ASSERT(frame.width == image.cols && frame.height == image.rows);
  Mat empty = Mat::zeros(frame.size(), CV_8UC1);

  Mat result_channel(frame.size(), CV_8UC3);
  Mat in[] = {image, empty, plane};
  int from_to[] = {0, 0, 1, 1, 2, 2};
  mixChannels(in, 3, &result_channel, 1, from_to, 3);
  return result_channel;
}

PointCloud<PointXYZRGBA> MyPointCloud::segmentedColor(cv::Mat frame_rgb,
                                                      cv::Mat P) {
  PointCloud<PointXYZRGBA> cloud;

  for (PointCloud<Point>::iterator pt = pc_.begin(); pt < pc_.end(); pt++) {
    Point2f xy = Velodyne::Velodyne::projectf(*pt, P);
    int val = frame_rgb.at<short>(cv::Point(xy.x, xy.y));
    int r = 0, g = 0, b = 0;
    switch (val) {
    case 1: {
      r = 70;
      g = 70;
      b = 70;
    } break;
    case 2: {
      r = 190;
      g = 153;
      b = 153;
    } break;
    case 3: {
      r = 250;
      g = 170;
      b = 160;
    } break;
    case 4: {
      r = 220;
      g = 20;
      b = 60;
    } break;
    case 5: {
      r = 153;
      g = 153;
      b = 153;
    } break;
    case 6: {
      r = 157;
      g = 234;
      b = 50;
    } break;
    case 7: {
      r = 128;
      g = 64;
      b = 128;
    } break;
    case 8: {
      r = 244;
      g = 35;
      b = 232;
    } break;
    case 9: {
      r = 107;
      g = 142;
      b = 35;
    } break;
    case 10: {
      r = 0;
      g = 0;
      b = 142;
    } break;
    case 11: {
      r = 102;
      g = 102;
      b = 156;
    } break;
    case 12: {
      r = 220;
      g = 220;
      b = 0;
    } break;
    }

    PointXYZRGBA rgba;
    rgba.x = pt->x;
    rgba.y = pt->y;
    rgba.z = pt->z;
    rgba.r = r;
    rgba.g = g;
    rgba.b = b;
    rgba.a = 255 - val;

    cloud.push_back(pt_rgba);
  }
  return cloud;
}
