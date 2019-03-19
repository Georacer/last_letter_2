#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <tf2/convert.h>

#include <tf2/convert.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <kdl/frames.hpp>

int main(int argc, char** argv){
  ros::init(argc, argv, "tf_listener");

  ros::NodeHandle node;

  tf2_ros::Buffer tfBuffer(ros::Duration(3.0),false);
  tf2_ros::TransformListener tfListener(tfBuffer);
  geometry_msgs::PointStamped v_in,v_out;
  v_in.header.stamp=ros::Time::now();
  v_in.point.x=0;
  v_in.point.y=0;
  v_in.point.z=0;

  ros::Rate rate(10.0);
  while (node.ok()){
    geometry_msgs::TransformStamped transformStamped;
    try{
      if (tfBuffer.canTransform("body_FLU", "inertial_NWU",ros::Time(0),ros::Duration(3.0)));
      transformStamped = tfBuffer.lookupTransform("body_FLU", "inertial_NWU", ros::Time(0));
      tf2::doTransform(v_in, v_out, transformStamped);
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
    printf("x=%f  y=%f  z=%f\n",v_out.point.x,v_out.point.y,v_out.point.z);
    // rate.sleep();
  }
  return 0;
};