#include "ros/ros.h"
#include "std_msgs/String.h"
#include <laser_geometry/laser_geometry.h>
#include <sstream>

//Helpful:
// http://wiki.ros.org/pcl
// http://wiki.ros.org/pcl/Overview
// http://wiki.ros.org/pcl/Tutorials
// https://pcl.readthedocs.io/projects/tutorials/en/latest/index.html
// https://docs.ros.org/en/diamondback/api/laser_geometry/html/laser__geometry_8cpp_source.html
// https://github.com/koide3/ndt_omp/blob/master/apps/align.cpp

// http://wiki.ros.org/pcl/Tutorials/hydro?action=AttachFile&do=view&target=example_planarsegmentation.cpp
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>

// https://pcl.readthedocs.io/projects/tutorials/en/latest/iterative_closest_point.html
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  //-->Create a container for the data.
  //sensor_msgs::PointCloud2 output;
  //-->Do data processing here...
  //output = *input;
  //-->Publish the data.
  //pub.publish (output);

  //ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lasertest");
  ros::NodeHandle n;

  //ros::Publisher pub = n.advertise<std_msgs::String>("chatter", 1000);
  ros::Subscriber sub = n.subscribe("/scan", 1000, cloudCallback);

  ros::Rate loop_rate(10);
  //int count = 0;
  while (ros::ok())
  {
    //std_msgs::String msg;
    //std::stringstream ss;
    //ss << "hello world " << count;
    //msg.data = ss.str();
    //ROS_INFO("%s", msg.data.c_str());
    //pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    //++count;
  }
  return 0;
}

