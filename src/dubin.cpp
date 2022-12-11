
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

#include "nav2_msgs/action/follow_path.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "dubin.h"
#include "PRMstar.h"
#include "map.h"
#include "dubinCurve.h"

using namespace std::chrono_literals;


pose2d Dubin::subscribeToPos(){
  std::string target_frame_ = this->declare_parameter<std::string>("target_frame", "gazebo/base_link");
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
  /*
    PUBLISHER AND SUBSCRIBER PART 
  */

  print_message();
  float delta = 0.01;
  pose2d x0 = this->subscribeToPos();
  //pose2d x0(0.1,0.1,4.0);
  pose2d x1(0.0,0.0,0.0);

  
  // Map* map = new Map(0.0, 0.0, 10.0, 10.0);
  // planner = new PRMstar(map);
  // planner->genRoadmap(100);
  // std::vector<point2d> mids = planner->getPath(x0.x,x1.x);
  // if (mids.size() >= 2)
  // {
  //   mids.pop_back();
  //   mids.erase(mids.begin());
  // }

  std::vector<point2d> mids;
  //// RCLCPP_INFO(this->get_logger(),"psd");
  dubinCurve d;
  d._K = 3;
  nav_msgs::msg::Path message =  d.generatePathFromDubins(x0, d.calculateMultiPoint(x0, x1, mids, 3), delta);
  message.header.stamp = this->get_clock()->now();

  // publisher_->publish(message);
  // RCLCPP_INFO(this->get_logger(),"message shoud have sent!");


  /*
      ACTION CLIENT AND SERVER PART
  */

  using FollowPath = nav2_msgs::action::FollowPath;

  rclcpp_action::Client<FollowPath>::SharedPtr client_ptr_;
  
  client_ptr_ = rclcpp_action::create_client<FollowPath>(this,"follow_path");
  
  if (!client_ptr_->wait_for_action_server()) {
    RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
    rclcpp::shutdown();
  }

  auto goal_msg = FollowPath::Goal();
  goal_msg.path = message;
  goal_msg.controller_id = "FollowPath";

  RCLCPP_INFO(this->get_logger(), "Sending goal position");
  client_ptr_->async_send_goal(goal_msg);

  for(int i = 0; i<2; i++)
  {
      publisher_->publish(message);
      usleep(1000000);
      RCLCPP_INFO(this->get_logger(), "%s", message.header.frame_id.c_str());
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Dubin>());
  rclcpp::shutdown();
  return 0;
};

