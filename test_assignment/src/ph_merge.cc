#define DEBUG 1

#include <ros/ros.h>
#include "ros/package.h"
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl_ros/transforms.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/CameraInfo.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

using namespace std;
using namespace sensor_msgs;

typedef pcl::PointCloud<pcl::PointXYZRGBA> PointCloudXYZRGBA;
ros::Publisher pcl_pub;
string target_frame, source_frame;
  tf::StampedTransform transform_;

void getLabel(int &r,int& g, int &b,int val) {

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

}

void callback(const PointCloud2::ConstPtr& pcl_msg, const CameraInfoConstPtr& cinfo_msg, const ImageConstPtr& image_msg){
  if(DEBUG) ROS_INFO("\n\nColouring VELODYNE CLOUD!!");;
  // Conversion
  cv_bridge::CvImageConstPtr cv_img_ptr;
  cv::Mat image;
  try{
    image = cv_bridge::toCvShare(image_msg,"mono16")->image;
  }catch (cv_bridge::Exception& e){
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  image_geometry::PinholeCameraModel cam_model_;
  cam_model_.fromCameraInfo(cinfo_msg);

  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>); // From ROS Msg
  pcl::PointCloud<pcl::PointXYZ>::Ptr trans_cloud(new pcl::PointCloud<pcl::PointXYZ>); // After transformation
  PointCloudXYZRGBA::Ptr coloured = PointCloudXYZRGBA::Ptr(new PointCloudXYZRGBA); // For coloring purposes
  fromROSMsg(*pcl_msg, *pcl_cloud);

  

  if(DEBUG) cout << "FRAME ID "<< pcl_cloud->header.frame_id << endl;

  pcl_ros::transformPointCloud (*pcl_cloud, *trans_cloud, transform_);
  trans_cloud->header.frame_id = target_frame;

  pcl::copyPointCloud(*trans_cloud, *coloured);

  for (pcl::PointCloud<pcl::PointXYZRGBA>::iterator pt = coloured->points.begin(); pt < coloured->points.end(); pt++)
  {
    cv::Point3d pt_cv((*pt).x, (*pt).y, (*pt).z);
    cv::Point2d uv;
    uv = cam_model_.project3dToPixel(pt_cv);

    if(uv.x>0 && uv.x < image.cols && uv.y > 0 && uv.y < image.rows){
      int r = 0, g = 0, b = 0;

      int val = image.at<short>(cv::Point(uv.x, uv.y)); 
      getLabel(r,g,b,val);
      // Copy colour to laser pointcloud
      (*pt).b = b;
      (*pt).g = g;
      (*pt).r = r;
      (*pt).a = 255 - val;
    }

  }

  if(DEBUG) ROS_INFO("Publish coloured PC");

  // Publish coloured PointCloud
  sensor_msgs::PointCloud2 pcl_colour_ros;
  pcl::toROSMsg(*coloured, pcl_colour_ros);
  pcl_colour_ros.header.stamp = pcl_msg->header.stamp ;
  pcl_pub.publish(pcl_colour_ros);
}

int main(int argc, char **argv){
  ros::init(argc, argv, "pcl_coloring");
  ros::NodeHandle nh_("~"); // LOCAL
  // Parameters
  nh_.param<std::string>("target_frame", target_frame, "/camera_semseg");
  nh_.param<std::string>("source_frame", source_frame, "/lidar_0");

  // Subscribers
  message_filters::Subscriber<PointCloud2> pc_sub(nh_, "/lidar_0", 1);
  message_filters::Subscriber<CameraInfo> cinfo_sub(nh_, "/camera_semseg/camera_info", 1);
  message_filters::Subscriber<Image> image_sub(nh_, "/camera_semseg/image_raw", 1);

  pcl_pub = nh_.advertise<PointCloud2> ("velodyne_coloured", 1);

  typedef message_filters::sync_policies::ApproximateTime<PointCloud2, CameraInfo, Image> MySyncPolicy;
  // ExactTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), pc_sub, cinfo_sub, image_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2, _3));

tf::TransformListener listener;
  try{
    listener.waitForTransform(target_frame.c_str(), source_frame.c_str(), ros::Time(0), ros::Duration(20.0));
    listener.lookupTransform (target_frame.c_str(), source_frame.c_str(), ros::Time(0), transform_);
  }catch (tf::TransformException& ex) {
    ROS_WARN("[draw_frames] TF exception:\n%s", ex.what());
    return 0;
  }

  ros::spin();
  return 0;
}