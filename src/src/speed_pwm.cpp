#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int16.h"

#include <sstream>

std_msgs::Int16 speed_to_pub;
std_msgs::Int16 off_speed_to_pub;
double percent = 1.0;
double rate = 100;
int max_count = 8;

void speedCallback(const std_msgs::Int16& msg) {
  int speed = (double) msg.data;
  if (speed < 1100) {
    speed_to_pub.data = 1100;
  } else if (speed > 1900) {
    speed_to_pub.data = 1900;
  } else {
    speed_to_pub.data = speed;
  }
  std::cout << speed << std::endl;
}

void maxCallback(const std_msgs::Int16& msg) {
  max_count = msg.data;
  std::cout << max_count << std::endl;
}

void percentCallback(const std_msgs::Float64& msg) {
  percent = (double) msg.data;
  if (percent < 0) {
    percent = 0.0;
  } else if (percent > 1) {
    percent = 1.0;
  }
  std::cout << percent << std::endl;
}

void freqCallback(const std_msgs::Float64& msg) {
  rate = (double) msg.data;
  if (rate < 0) {
    rate = 0.0;
  } else if (rate > 150) {
    rate = 150;
  }
  std::cout << rate << std::endl;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<std_msgs::Int16>("/auto_cmd/throttle", 1);
  ros::Subscriber sub_speed = n.subscribe("/auto_cmd/raw_throttle", 1, speedCallback);
  ros::Subscriber sub_percent = n.subscribe("/pwm/percent", 1, percentCallback);
  ros::Subscriber sub_freq = n.subscribe("/pwm/freq", 1, freqCallback);
  ros::Subscriber sub_max = n.subscribe("/pwm/max", 1, maxCallback);
  auto rateObj = ros::Rate(rate);
  ROS_INFO("Listening...");

  speed_to_pub.data = 1500;
  off_speed_to_pub.data = 1500;
  int count = 0;

  while (ros::ok()) {
    rateObj = ros::Rate(rate);
    if (max_count * percent > count) {
      //std::cout << percent << " " << speed_to_pub.data << " " << count << std::endl;
      pub.publish(speed_to_pub);
    } else {
      //std::cout << percent << " " << off_speed_to_pub.data << " " << count << std::endl;
      pub.publish(off_speed_to_pub);
    }
    ros::spinOnce();
    rateObj.sleep();
    count = (count + 1) % max_count;
  }
  speed_to_pub.data = 1500;
  pub.publish(speed_to_pub);

  return 0;
}

