#pragma once
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

#include "helpers.h"
#include "PRMstar.h"

using namespace std::chrono_literals;

class dubinCurve
{
  public:
  float _K;
  float mod2pi(float theta); // DONE
  float sinc(float x);

  geometry_msgs::msg::PoseStamped pose2dToPose(pose2d pose);
  pose2d nextPointOnArc(pose2d x0, float ds, int k);

  nav_msgs::msg::Path generatePathFromDubins
  (pose2d start, std::vector<dubins_params> sub_paths, float delta);
  std::vector<dubins_params> calculateMultiPoint(pose2d start, pose2d end, std::vector<point2d> mid_points, int n_angles);
  // Scale the input problem to standard form (x0: -1, y0: 0, xf: 1, yf: 0)
  transformedVars scaleToStandard(pose2d x0, pose2d x1);
  // Scale the solution to the standard problem back to the original problem
  dubins_params scaleFromStandard(float lambda, dubins_params params);
  
  dubins_params calculateSinglePath( pose2d x0, pose2d x1 );
  dubins_params DUBINATOR( float th0, float th1, float lambda, ksigns ks );

};

class Dubin : public rclcpp::Node
{

public:
  Dubin()
: Node("dubin_publisher"), count_(0)
{
  publisher_ = this->create_publisher<nav_msgs::msg::Path>("/plan", 10);
  // timer_ = this->create_wall_timer(
  //   500ms, std::bind(&Dubin::timer_callback, this));
  this->timer_callback();

};
  ~Dubin(){};

  pose2d subscribeToPos();

private:
  void timer_callback();
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_;
  size_t count_;

  
};


