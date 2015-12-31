#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <iostream>

#include <stdio.h>
#include <dlfcn.h>

extern "C" {
#include "lib.h"
}

namespace ros
{
  
  ros::Subscriber g_ros_start_glc_subscriber, g_ros_stop_glc_subscriber;

  void ros_start_glc_callback(const std_msgs::Empty::ConstPtr& empty_msg)
  {
    ROS_INFO("[%s] ros_start_glc", ros::this_node::getName().c_str());
    start_capture();
  }

  void ros_stop_glc_callback(const std_msgs::Empty::ConstPtr& empty_msg)
  {
    ros::NodeHandle nh;
    ROS_INFO("[%s] ros_stop_glc", ros::this_node::getName().c_str());
    start_capture();
  }
  
  void init(int& argc, char** argv, const std::string& name, uint32_t options)
  {
    
    typedef void (*fptr)(int& argc, char** argv, const std::string& name, uint32_t options);
    void (*o_init) (int& argc, char** argv, const std::string& name, uint32_t options);
    ROS_WARN("ros::init is overtaken by glc-capture");
    o_init = reinterpret_cast<fptr>(dlsym(RTLD_NEXT, "_ZN3ros4initERiPPcRKSsj")); // mangled ros::init
    if (o_init) {
      o_init(argc, argv, name, options);
      ros::NodeHandle n;
      g_ros_start_glc_subscriber = n.subscribe("/glc_start_capture", 1, ros_start_glc_callback);
      g_ros_stop_glc_subscriber = n.subscribe("/glc_stop_capture", 1, ros_stop_glc_callback);
    }
    else {
      std::cerr << "Failed to find original ros::init" << std::endl;
    }
    return;
  }
  
}
