
#include "ros/package.h"
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <opencv2/opencv.hpp>
#include <pcl/common/geometry.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/point_cloud2_iterator.h>

using namespace std;
using namespace sensor_msgs;

typedef pcl::PointCloud<pcl::PointXYZRGBA> PointCloudXYZRGBA;
ros::Publisher pcl_pub, dist_pub;
string target_frame, source_frame;
tf::StampedTransform lidar_cam_transform_,cam_baselink_transform_;

const static int MAX_VAL = 100000;
const static int ALPHA_OFFSET = 50;

// void updateMinDist(int index, vector<pcl::PointXYZRGBA> &minPoints,
//                    pcl::PointXYZRGBA origin, pcl::PointXYZRGBA new_point) {
//   int new_dist = pcl::geometry::distance(origin, new_point);
//   int old_dist = pcl::geometry::distance(origin, minPoints[index]);

//   if (new_dist < old_dist) {
//     minPoints[index].x = new_point.x;
//     minPoints[index].y = new_point.y;
//     minPoints[index].z = new_point.z;
//   }
// }

void getLabel(int &r, int &g, int &b, int val) {

  switch (val) {
  case 0: {
    r = 0;
    g = 0;
    b = 0;
  } break;
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
}

double euclideanDistance(const double &x1, const double &y1, const double &z1,
                         const double &x2, const double &y2, const double &z2) {
  double diff_x = x2 - x1, diff_y = y2 - y1, diff_z = z2 - z1;
  return sqrtf(diff_x * diff_x + diff_y * diff_y + diff_z * diff_z);
}

void labelledCallback(sensor_msgs::PointCloud2ConstPtr const &msg) {
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pcl_cloud(
      new pcl::PointCloud<pcl::PointXYZRGBA>); // From ROS Msg
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr trans_cloud(
      new pcl::PointCloud<pcl::PointXYZRGBA>); // After transformation
fromROSMsg(*msg, *pcl_cloud);
pcl_ros::transformPointCloud(*pcl_cloud, *trans_cloud, cam_baselink_transform_);
sensor_msgs::PointCloud2 pcl_colour_ros;
  pcl::toROSMsg(*trans_cloud, pcl_colour_ros);



  sensor_msgs::PointCloud2ConstIterator<float> iter_x(pcl_colour_ros, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(pcl_colour_ros, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(pcl_colour_ros, "z");
  sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_a(pcl_colour_ros, "a");

  visualization_msgs::Marker marker;
  marker.header.frame_id = "base_link";
  marker.header.stamp = ros::Time();
  marker.ns = "my_namespace";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 0.3;
  marker.scale.y = 0.3;
  marker.scale.z = 0.3;
  marker.color.a = 1.0; // Don't forget to set the alpha!
  marker.color.r = 1.0;
  marker.color.g = 1.0;
  marker.color.b = 1.0;
  std_msgs::ColorRGBA color;
  geometry_msgs::Point origin, far_point;
  far_point.x = far_point.y = far_point.z = MAX_VAL;
  vector<geometry_msgs::Point> min_points(13, far_point);

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_a) {
    int index = *iter_a - ALPHA_OFFSET;
    if(index<0 || index>12) ROS_INFO_STREAM(index);
    int old_dist = euclideanDistance(0, 0, 0, min_points[index].x,
                                     min_points[index].y, min_points[index].z);
    int new_dist = euclideanDistance(0, 0, 0, *iter_x, *iter_y, *iter_z);
    if(old_dist>new_dist) {
      min_points[index].x=*iter_x;
                                     min_points[index].y=*iter_y; min_points[index].z=*iter_z;
    }
  }

  for (int i = 0; i < min_points.size(); i++) {
    if (min_points[i].x == MAX_VAL || (i!=4 && i!=12))
      continue;
    marker.points.push_back(origin);
    marker.points.push_back(min_points[i]);
    int r = 0, g = 0, b = 0;
    getLabel(r, g, b, i);
    color.r = r;
    color.g = g;
    color.b = b;
    color.a = 1;
    marker.colors.push_back(color);
    marker.colors.push_back(color);
  }

  dist_pub.publish(marker);
}

void callback(const PointCloud2::ConstPtr &pcl_msg,
              const CameraInfoConstPtr &cinfo_msg,
              const ImageConstPtr &image_msg) {
  // Conversion
  cv_bridge::CvImageConstPtr cv_img_ptr;
  cv::Mat image;
  try {
    image = cv_bridge::toCvShare(image_msg)->image;
  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  image_geometry::PinholeCameraModel cam_model_;
  cam_model_.fromCameraInfo(cinfo_msg);

  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(
      new pcl::PointCloud<pcl::PointXYZ>); // From ROS Msg
  pcl::PointCloud<pcl::PointXYZ>::Ptr trans_cloud(
      new pcl::PointCloud<pcl::PointXYZ>); // After transformation
  PointCloudXYZRGBA::Ptr coloured =
      PointCloudXYZRGBA::Ptr(new PointCloudXYZRGBA); // For labelling purposes

  fromROSMsg(*pcl_msg, *pcl_cloud);

  ROS_INFO_STREAM_THROTTLE(1, "Frame: " << pcl_cloud->header.frame_id);

  pcl_ros::transformPointCloud(*pcl_cloud, *trans_cloud, lidar_cam_transform_);
  trans_cloud->header.frame_id = target_frame;

  pcl::copyPointCloud(*trans_cloud, *coloured);

  for (pcl::PointCloud<pcl::PointXYZRGBA>::iterator pt =
           coloured->points.begin();
       pt < coloured->points.end(); pt++) {
    cv::Point3d pt_cv((*pt).x, (*pt).y, (*pt).z);
    cv::Point2d uv;
    uv = cam_model_.project3dToPixel(pt_cv);

    if (uv.x > 0 && uv.x < image.cols && uv.y > 0 && uv.y < image.rows &&
        (*pt).y < 0) {
      int r = 0, g = 0, b = 0;

      int val = image.at<short>(cv::Point(uv.x, uv.y));
      getLabel(r, g, b, val);
      // Copy colour to laser pointcloud
      (*pt).b = b;
      (*pt).g = g;
      (*pt).r = r;
      (*pt).a = val + ALPHA_OFFSET;
    }
  }

  // Publish coloured PointCloud
  sensor_msgs::PointCloud2 pcl_colour_ros;
  pcl::toROSMsg(*coloured, pcl_colour_ros);
  pcl_colour_ros.header.stamp = pcl_msg->header.stamp;
  pcl_pub.publish(pcl_colour_ros);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "pcl_coloring");
  ros::NodeHandle nh_("~"); // LOCAL
  // Parameters
  nh_.param<std::string>("target_frame", target_frame, "/camera_semseg");
  nh_.param<std::string>("source_frame", source_frame, "/lidar_0");

  // Subscribers
  message_filters::Subscriber<PointCloud2> pc_sub(nh_, "/lidar_0", 1);
  message_filters::Subscriber<CameraInfo> cinfo_sub(
      nh_, "/camera_semseg/camera_info", 1);
  message_filters::Subscriber<Image> image_sub(nh_, "/camera_semseg/image_raw",
                                               1);

  ros::Subscriber sub_lab_pc = nh_.subscribe("labelled_pc", 1, labelledCallback);

  pcl_pub = nh_.advertise<PointCloud2>("labelled_pc", 1);
  dist_pub = nh_.advertise<visualization_msgs::Marker>("dist_marker", 1);

  typedef message_filters::sync_policies::ApproximateTime<PointCloud2,
                                                          CameraInfo, Image>
      MySyncPolicy;
  // ExactTime takes a queue size as its constructor argument, hence
  // MySyncPolicy(10)
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), pc_sub,
                                                   cinfo_sub, image_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2, _3));

  tf::TransformListener listener;
  try {
    listener.waitForTransform(target_frame.c_str(), source_frame.c_str(),
                              ros::Time(0), ros::Duration(20.0));
    listener.lookupTransform(target_frame.c_str(), source_frame.c_str(),
                             ros::Time(0), lidar_cam_transform_);
  } catch (tf::TransformException &ex) {
    ROS_WARN("TF exception:\n%s", ex.what());
    return 0;
  }

  try {
    listener.waitForTransform("base_link", target_frame.c_str(),
                              ros::Time(0), ros::Duration(20.0));
    listener.lookupTransform("base_link", target_frame.c_str(),
                             ros::Time(0), cam_baselink_transform_);
  } catch (tf::TransformException &ex) {
    ROS_WARN("TF exception:\n%s", ex.what());
    return 0;
  }

  ros::spin();
  return 0;
}