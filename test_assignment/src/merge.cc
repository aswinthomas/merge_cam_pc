
// opencv
#include <cv_bridge/cv_bridge.h>

// pcl
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>

// ros
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <string>
#include <typeinfo>
#include <image_transport/image_transport.h>

// internal
#include <test_assignment/MyPointCloud.h>

cv::Mat proj_mat_;
cv::Mat image_;
std::vector<float> calibration_;
ros::Publisher pub_pc_;


using namespace std;
using namespace cv;
using namespace pcl;

void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &msg) {
  float p[12];
  float *pp = p;
  for (boost::array<double, 12ul>::const_iterator i = msg->P.begin();
       i != msg->P.end(); i++) {
    *pp = (float)(*i);
    pp++;
  }

  cv::Mat(3, 4, CV_32FC1, &p).copyTo(proj_mat_);
}

void imageCallback(const sensor_msgs::ImageConstPtr &msg) {
  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg);
  image_ = cv_ptr->image;
}

void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg) {
  if (image_.data == NULL)
    return;

  PointCloud<MyPoint> pc;
  fromROSMsg(*msg, pc);

  // transform to camera coordinates
  MyPointCloud pointcloud = MyPointCloud(pc).transform(0, 0, 0, M_PI / 2, 0, 0);

  MyPointCloud transformed = pointcloud.transform(calibration_);
  PointCloud<MyPoint> filtered_pc;
  transformed.project(proj_mat_, Rect(0, 0, image_.cols, image_.rows),
                      &filtered_pc);

  MyPointCloud visible_scan(filtered_pc);

  PointCloud<PointXYZRGBA> color_cloud =
      visible_scan.segmentedColor(image_, proj_mat_);

  // transform back
  Eigen::Affine3f transf = getTransformation(0, 0, 0, -M_PI / 2, 0, 0);
  transformPointCloud(color_cloud, color_cloud, transf);

  sensor_msgs::PointCloud2 color_cloud2;
  toROSMsg(color_cloud, color_cloud2);
  color_cloud2.header = msg->header;

  pub_pc_.publish(color_cloud2);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "pc_labeler");

  ros::NodeHandle n;
  pub_pc_ = n.advertise<sensor_msgs::PointCloud2>("labelled_pc", 1);

  // Subscribe input camera image
  image_transport::ImageTransport it(n);
  image_transport::Subscriber sub =
      it.subscribe("/camera_semseg/image_raw", 10, imageCallback);
  ros::Subscriber info_sub =
      n.subscribe("/camera_semseg/camera_info", 10, cameraInfoCallback);
  ros::Subscriber pc_sub =
      n.subscribe<sensor_msgs::PointCloud2>("/lidar_0", 1, pointCloudCallback);
  n.getParam("calibration", calibration_);
  ros::spin();

  return EXIT_SUCCESS;
}