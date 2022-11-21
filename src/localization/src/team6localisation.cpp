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

/////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////// Defines, Globals, Constructors //////////////////////////////

//// Defines:
#define FRAME_ID ("map")
#define DEBUG (1)

//// Global variables:
pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>); // PointCloud from Laser
pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>); // PointCloud from World Map
Eigen::Matrix4f initpose = Eigen::Matrix4f::Identity(); // Initial pose guess for localisation algorithms, nonhomogenous transform
bool scan_received = false; // Flag whether a new scan pointcloud has been received
bool grid_received = false; // Flag whether a reference grid pointcloud has been received

//// ROS Callback Function Constructors:
void scanCallback(const sensor_msgs::PointCloud2ConstPtr& msg);
void gridCallback(const sensor_msgs::PointCloud2ConstPtr& msg);
void initposeCallback(const geometry_msgs::PoseWithCovarianceStamped& msg);

/////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////// ROS Callback Functions //////////////////////////////////

void scanCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
  // This function toggles the scan_received flag and processes an inbound pc to PCL format
  // Input: ROS PointCloud Pointer
  // Output: None (Global Variable Updated)
  if (!scan_received) {
    //ROS_INFO("Scan received.");
  }
  pcl::fromROSMsg(*msg, *source_cloud);
  scan_received = true;
}

void gridCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
  // This function toggles the grid_received flag and processes an inbound pc to PCL format
  // Input: ROS PointCloud Pointer
  // Output: None (Global Variable Updated)
  if (!grid_received) {
    //ROS_INFO("Grid received.");
  }
  pcl::fromROSMsg(*msg, *target_cloud);
  grid_received = true;
}

void initposeCallback(const geometry_msgs::PoseWithCovarianceStamped& msg) { 
  // This function updates the initpose matrix with a new estimate
  // Input: ROS PoseWithCovarianceStamped
  // Output: None (Global Variable Updated)

  // quaternion -> rotation mat 
  Eigen::Quaterniond quat_mat(msg.pose.pose.orientation.w, 
                              msg.pose.pose.orientation.x, 
                              msg.pose.pose.orientation.y, 
                              msg.pose.pose.orientation.z);

  Eigen::Matrix3d rotd_mat = quat_mat.normalized().toRotationMatrix();
  Eigen::Matrix3f rotf_mat = rotd_mat.cast <float> ();

  //std::cout << "R=" << std::endl << rotf_mat << std::endl;

  Eigen::Vector3f trnl_mat({(float) msg.pose.pose.position.x, 
                            (float) msg.pose.pose.position.y, 
                            0}); // msg->pose.pose.position.z});

  Eigen::Matrix4f nhmg_mat; // nonhomogenous transform

  // populate transformation matrix:
  nhmg_mat.setIdentity();
  nhmg_mat.block<3,3>(0,0) = rotf_mat;
  nhmg_mat.block<3,1>(0,3) = trnl_mat;

  //std::cout << "T=" << std::endl << nhmg_mat << std::endl;
  initpose = nhmg_mat; // Store in global variable
}

/////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////// Main Loop /////////////////////////////////////////

int main(int argc, char **argv) {

  ros::init(argc, argv, "team6localisation_node");
  ros::NodeHandle n;

  ROS_INFO("PointCloud Alignment node ready. Spinning.");

  //// Set up ROS subscribers, publishers:
  //ros::Subscriber sub_scan = n.subscribe("/scan2pc_pre", 1, scanCallback);
  ros::Subscriber sub_scan = n.subscribe("/scan_grid2pc", 1, scanCallback); // get new scan PC
  ros::Subscriber sub_grid = n.subscribe("/map_grid2pc", 1, gridCallback); // get new grid PC
  ros::Subscriber sub_pose = n.subscribe("/initialpose", 1, initposeCallback); // listen for new initial guesses
  ros::Publisher pub_pose = n.advertise<geometry_msgs::PoseWithCovariance>("/localised_pose", 1); // just pose to pub
  ros::Publisher pub_odom = n.advertise<nav_msgs::Odometry>("/localised_odom", 1); // odom to pub; pose embedded
  ros::Publisher pub_ptcd = n.advertise<sensor_msgs::PointCloud2>("/pc_localised", 1); // aligned pc to pub for diagnostics

  ros::Rate loop_rate(40); // 40 Hz; maximum of laserscanner publish rate
  while(ros::ok()) { // while ROS hasn't died and roscore connection behaving correctly:
    if (scan_received && grid_received && DEBUG == 1) { // ensure we have a grid of each to compare
      //ROS_INFO("New scan ready to be processed!");

      // // downsampling, voxelisation:
      // pcl::VoxelGrid<pcl::PointXYZ> voxelgrid;
      // voxelgrid.setLeafSize(0.05f, 0.05f, 0.05f); // same resolution as occupancygrid

      // pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_target(new pcl::PointCloud<pcl::PointXYZ>);
      // voxelgrid.setInputCloud(target_cloud);
      // voxelgrid.filter(*downsampled_target);
      // target_cloud = downsampled_target;

      // pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_source(new pcl::PointCloud<pcl::PointXYZ>);
      // voxelgrid.setInputCloud(source_cloud);
      // voxelgrid.filter(*downsampled_source);
      // source_cloud = downsampled_source;

      ros::Time::init();

      //// NDT_OMP Localisation Implementation:
      pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt_omp;
      ndt_omp.setResolution(1.0);

      pcl::PointCloud<pcl::PointXYZ> aligned;
      ndt_omp.setNumThreads(omp_get_max_threads());
      ndt_omp.setNeighborhoodSearchMethod(pclomp::DIRECT7);
      ndt_omp.setInputTarget(target_cloud);
      ndt_omp.setInputSource(source_cloud);

      ndt_omp.align(aligned, initpose); // feed in initial guess
      //std::cout << "has converged:" << ndt_omp.hasConverged() << " score: " << ndt_omp.getFitnessScore() << std::endl;
      initpose = ndt_omp.getFinalTransformation(); // update global variable for next iteration
      //std::cout << nhmg_mat << std::endl;

      //// Convert nonhomogenous transformation to quaternion and position vector
      Eigen::Matrix3f rot_mat = initpose.block<3,3>(0,0); // extract rot matrix
      Eigen::Vector3f trl_mat = initpose.block<3,1>(0,3); // extract translation matrix
      Eigen::Quaternionf quat_mat(rot_mat);

      //// Prepare ROS Messages for publishing:
      // Pose:
      geometry_msgs::PoseWithCovariance pose_to_pub;
      pose_to_pub.pose.position.x = trl_mat[0];
      pose_to_pub.pose.position.y = trl_mat[1];
      pose_to_pub.pose.position.z = 0;//trl_mat[2];
      pose_to_pub.pose.orientation.x = quat_mat.x();
      pose_to_pub.pose.orientation.y = quat_mat.y();
      pose_to_pub.pose.orientation.z = quat_mat.z();
      pose_to_pub.pose.orientation.w = quat_mat.w();

      // Odom (with Pose embedded):
      nav_msgs::Odometry odom_to_pub;
      odom_to_pub.pose = pose_to_pub;
      odom_to_pub.child_frame_id = FRAME_ID;
      odom_to_pub.header.frame_id = FRAME_ID;
      odom_to_pub.header.stamp = ros::Time();

      // PointCloud:
      sensor_msgs::PointCloud2 pc_to_pub;
      pcl::toROSMsg(aligned, pc_to_pub);
      pc_to_pub.header.frame_id = FRAME_ID;

      // Publish:
      pub_ptcd.publish(pc_to_pub);
      pub_pose.publish(pose_to_pub);
      pub_odom.publish(odom_to_pub);

      scan_received = false;
    }
    ros::spinOnce();
    loop_rate.sleep();

  }
  return 0;
}

/////////////////////////////////////////////////////////////////////////////////////////////
/////////// Old Includes, Helpful Websites, Alternative Algorithm Implementations ///////////

//// Old Includes:
      //#include "std_msgs/String.h"
      //#include "std_msgs/Header.h"
      //#include "std_msgs/Time.h"
      //#include "std_msgs/UInt32.h"
      //#include "std_msgs/Float64.h"
      //#include "geometry_msgs/PoseStamped.h"
      //#include "geometry_msgs/Pose.h"
      //#include "geometry_msgs/TwistWithCovarianceStamped.h"
      //#include "geometry_msgs/TwistWithCovariance.h"
      //#include "geometry_msgs/TwistStamped.h"
      //#include "geometry_msgs/Twist.h"
      //#include "geometry_msgs/Quaternion.h"
      //#include "geometry_msgs/Point.h"
      //#include <laser_geometry/laser_geometry.h>
      //#include <pcl/registration/icp.h>
      //#include <pcl/registration/ndt.h>
      //#include <pcl/registration/gicp.h>
      //#include <pclomp/gicp_omp.h>
      //#include <pcl/conversions.h>
      //#include <pcl/sample_consensus/model_types.h>
      //#include <pcl/sample_consensus/method_types.h>
      //#include <pcl/segmentation/sac_segmentation.h>

//// Helpful Websites (and references for this code):
      // http://wiki.ros.org/pcl
      // http://wiki.ros.org/pcl/Overview
      // http://wiki.ros.org/pcl/Tutorials
      // http://wiki.ros.org/pcl/Tutorials/hydro?action=AttachFile&do=view&target=example_planarsegmentation.cpp
      // https://pcl.readthedocs.io/projects/tutorials/en/latest/index.html
      // https://pcl.readthedocs.io/projects/tutorials/en/latest/iterative_closest_point.html
      // https://docs.ros.org/en/diamondback/api/laser_geometry/html/laser__geometry_8cpp_source.html
      // https://github.com/koide3/ndt_omp/blob/master/apps/align.cpp
      // https://gist.github.com/LimHyungTae/2499a68ea8ee4d8a876a149858a5b08e

//// Alternative Algorithm Implementations:
      // pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
      // icp.setInputTarget(target_cloud);
      // icp.setInputSource(source_cloud);
      
      // pcl::PointCloud<pcl::PointXYZ> aligned;
      // for(int i=0; i<10; i++) { // upped to 40 from 10
      //   icp.align(aligned, initpose);
      // }
      // std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
      // std::cout << icp.getFinalTransformation() << std::endl;

      // pclomp::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp_omp;
      // gicp_omp.setInputTarget(target_cloud);
      // gicp_omp.setInputSource(source_cloud);
      
      // pcl::PointCloud<pcl::PointXYZ> aligned;
      // for(int i=0; i<10; i++) { // upped to 40 from 10
      //   gicp_omp.align(aligned, initpose);
      // }
      // std::cout << "has converged:" << gicp_omp.hasConverged() << " score: " << gicp_omp.getFitnessScore() << std::endl;
      // std::cout << gicp_omp.getFinalTransformation() << std::endl;