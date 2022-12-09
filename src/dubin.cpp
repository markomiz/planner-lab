
#include <chrono>
#include <memory>
#include <vector>
#include <math.h>
#include <iostream>
#include <fstream>
#include <unistd.h>
#include <string>

#include "tf2/exceptions.h"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "dubin.h"
#include "PRMstar.h"
#include "map.h"
#include "dubinCurve.h"
#include "config_server.h"
using namespace std::chrono_literals;


pose2d Dubin::subscribeToPos(){
  std::string target_frame_ = this->declare_parameter<std::string>("target_frame", "shelfino2/base_link");
  std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer;
  tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
  std::string fromFrameRel = target_frame_.c_str();
  std::string toFrameRel = std::string("map");
  geometry_msgs::msg::TransformStamped t;
  pose2d xyth(0,0,0);

  try {
  t = tf_buffer->lookupTransform(toFrameRel, fromFrameRel, tf2::TimePointZero, 5000ms);
  } catch (const tf2::TransformException & ex) {
  return xyth;
  }
  tf2::Quaternion q(t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);


  xyth.x.x  = t.transform.translation.x;
  xyth.x.y  = t.transform.translation.y;
  xyth.theta = yaw;
  return xyth;
};


void Dubin::timer_callback()
{

  print_message();
  float delta = 0.01;
  pose2d x0(0.1,0.1,4.0);
  pose2d x1(3.0,5.0,0.0);
  shared_ptr<Map> map (new Map(0.0, 0.0, 10.0, 10.0));
  RCLCPP_INFO(this->get_logger()," gmap made");
  shared_ptr<dubinCurve> d (new dubinCurve());
  d->map = map;
  d->_K = conf->getK();
  planner = new PRMstar(map);
  RCLCPP_INFO(this->get_logger()," planner made");
  planner->dCurve = d;
  planner->genRoadmapPlus(conf->getNumPoints(), conf->getNumAngles());
  RCLCPP_INFO(this->get_logger()," gen roadmap!");
  std::vector<arcs> mids = planner->getPath(x0,x1);
  RCLCPP_INFO(this->get_logger()," got path!");
  //nav_msgs::msg::Path message =  d->generatePathFromDubins(x0, d->calculateMultiPoint(x0, x1, mids, 12), delta);
  // message.header.stamp = this->get_clock()->now();
  // publisher_->publish(message);
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Dubin>());
  rclcpp::shutdown();
  return 0;
};

