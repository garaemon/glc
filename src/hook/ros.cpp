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
    ROS_INFO("ros_start_glc");
    start_capture();
  }

  void ros_stop_glc_callback(const std_msgs::Empty::ConstPtr& empty_msg)
  {
    ROS_INFO("ros_stop_glc");
    start_capture();
  }
  
  void init(int& argc, char** argv, const std::string& name, uint32_t options)
  {
    
    typedef void (*fptr)(int& argc, char** argv, const std::string& name, uint32_t options);
    void (*o_init) (int& argc, char** argv, const std::string& name, uint32_t options);
    std::cerr << "my init" << std::endl;
    //o_init = reinterpret_cast<fptr>(dlsym(RTLD_NEXT, "ros::init"));
    o_init = reinterpret_cast<fptr>(dlsym(RTLD_NEXT, "_ZN3ros4initERiPPcRKSsj"));
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
