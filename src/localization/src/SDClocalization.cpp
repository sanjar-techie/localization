
///////////////////////////////  includes
#include <iostream>
#include <sstream>
#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <tf/tf.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/PointCloud2.h"
#include <pclomp/ndt_omp.h>
#include <pcl_conversions/pcl_conversions.h>



///////////////////////////////  declare global variables
pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_source(new pcl::PointCloud<pcl::PointXYZ>); ///// PointCloud LaserScan
pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_target(new pcl::PointCloud<pcl::PointXYZ>); ///// PointCloud OccupancyGrid

Eigen::Matrix4f localization = Eigen::Matrix4f::Identity();


///////////////////////////////  flags
bool flag_lasorscan = FALSE; 
bool flag_occupancygrid = FALSE; 


///////////////////////////////  callbacks
void LASORSCAN_Callback(const sensor_msgs::PointCloud2ConstPtr& msg);
void OCCUPANCYGRID_Callback(const sensor_msgs::PointCloud2ConstPtr& msg);
void LOCALIZATION_Callback(const geometry_msgs::PoseWithCovarianceStamped& msg);


void LASORSCAN_Callback(const sensor_msgs::PointCloud2ConstPtr& msg) {

  if (flag_lasorscan == TRUE) {
    ROS_INFO("LASORSCAN_READY");
  }
    flag_lasorscan = TRUE;

    pcl::fromROSMsg(*msg, *pointcloud_source);
}

void OCCUPANCYGRID_Callback(const sensor_msgs::PointCloud2ConstPtr& msg) {
  if (flag_occupancygrid == TRUE) {
    ROS_INFO("OCCUPANCYGRID_READY");
  }
  grid_received = TRUE;
  
  pcl::fromROSMsg(*msg, *pointcloud_target);
}


void LOCALIZATION_Callback(const geometry_msgs::PoseWithCovarianceStamped& msg) { 

  ///// form nonhomogenous transform
  Eigen::Quaterniond quaternion_double(msg.pose.pose.orientation.w, 
                              msg.pose.pose.orientation.x, 
                              msg.pose.pose.orientation.y, 
                              msg.pose.pose.orientation.z);

  Eigen::Matrix3d rotation_double = quat_mat.normalized().toRotationMatrix();
  Eigen::Matrix3f rotation_float = rotd_mat.cast <float> ();


  Eigen::Vector3f translation_float({(float) msg.pose.pose.position.x, 
                            (float) msg.pose.pose.position.y, 
                            0}); ///// 2D

  Eigen::Matrix4f nonhomogenous;

  nonhomogenous.setIdentity();
  nonhomogenous.block<3,3>(0,0) = rotation_float;
  nonhomogenous.block<3,1>(0,3) = translation_float;

  localization = nonhomogenous; ///// ready for ndt_omp
}



/////////////////////////////////////
///////////////////////////////  main

int main(int argc, char **argv) {

  ros::init(argc, argv, "SDClocalization_node");
  ros::NodeHandle n;

///////////////////////////////  ros subscribers and publishers

  ros::Subscriber sub_scan = n.subscribe("/scan_grid2pc", 1, LASORSCAN_Callback); 
  ros::Subscriber sub_grid = n.subscribe("/map_grid2pc", 1, OCCUPANCYGRID_Callback); 
  ros::Subscriber sub_pose = n.subscribe("/initialpose", 1, LOCALIZATION_Callback); 

  ros::Publisher pub_pose = n.advertise<geometry_msgs::PoseWithCovariance>("/pose_20190338", 1); 
  ros::Publisher pub_odom = n.advertise<nav_msgs::Odometry>("/odom_20190338", 1); 
  ros::Publisher pub_ptcd = n.advertise<sensor_msgs::PointCloud2>("/pointcloud_20190338", 1);


///////////////////////////////  ros subscribers and publishers
  ros::Rate loop_rate(40); ///// laserscan rate
  
  while(ros::ok()) {
    if (flag_lasorscan && flag_occupancygrid) {
      ROS_INFO("LOCALIZATION READY");

      ros::Time::init();

      ////////////////////////////////  NDT_OMP
      pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> NDT_OMP;
      NDT_OMP.setResolution(1.0);

      pcl::PointCloud<pcl::PointXYZ> aligned;
      NDT_OMP.setNumThreads(omp_get_max_threads());
      NDT_OMP.setNeighborhoodSearchMethod(pclomp::DIRECT7);

      NDT_OMP.setInputTarget(pointcloud_target);
      NDT_OMP.setInputSource(pointcloud_source);

      NDT_OMP.align(aligned, localization); 

      localization = NDT_OMP.getFinalTransformation(); 



      Eigen::Matrix3f rotation = initpose.block<3,3>(0,0); 
      Eigen::Vector3f translation = initpose.block<3,1>(0,3);
      
      Eigen::Quaternionf quaternion(rotation);


      //// Prepare ROS Messages for publishing:
      // Pose:
      geometry_msgs::PoseWithCovariance pose_to_pub;

      pose_to_pub.pose.position.x = translation[0];
      pose_to_pub.pose.position.y = translation[1];
      pose_to_pub.pose.position.z = 0;

      pose_to_pub.pose.orientation.x = quaternion.x();
      pose_to_pub.pose.orientation.y = quaternion.y();
      pose_to_pub.pose.orientation.z = quaternion.z();
      pose_to_pub.pose.orientation.w = quaternion.w();

      // Odom (with Pose embedded):
      nav_msgs::Odometry odom_to_pub;

      odom_to_pub.pose = pose_to_pub;
      odom_to_pub.child_frame_id = "map";
      odom_to_pub.header.frame_id = "map";
      odom_to_pub.header.stamp = ros::Time();

      // PointCloud:
      sensor_msgs::PointCloud2 pc_to_pub;
      pcl::toROSMsg(aligned, pc_to_pub);
      pc_to_pub.header.frame_id = "map";

      // Publish:
      pub_ptcd.publish(pc_to_pub);
      pub_pose.publish(pose_to_pub);
      pub_odom.publish(odom_to_pub);

      flag_lasorscan = FALSE;
    }

    ros::spinOnce();
    loop_rate.sleep();

  }
  return 0;
}

