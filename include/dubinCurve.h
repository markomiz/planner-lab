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

#include "geometry.h"
#include "graph.h"
#include "Planner.h"
#include <deque>

using namespace std::chrono_literals;

class dubinCurve
{
  public:
  float _K; // maybe get rid of this?

  nav_msgs::msg::Path generatePathFromDubins(pose2d start, std::vector<dubins_params> sub_paths, float delta);
  nav_msgs::msg::Path arcs_to_path(deque<arcs> input_arcs, float delta);
  std::deque<arcs> calculateMultiPoint(pose2d start, pose2d end, std::deque<point2d> mid_points, int n_angles);

  // Scale the input problem to standard form (x0: -1, y0: 0, xf: 1, yf: 0)
  transformedVars scaleToStandard(pose2d x0, pose2d x1);
  
  // Scale the solution to the standard problem back to the original problem
  dubins_params scaleFromStandard(float lambda, dubins_params params);
  
  dubins_params calculateSinglePath( pose2d x0, pose2d x1 );
  dubins_params DUBINATOR( float th0, float th1, float lambda, ksigns ks );

  shared_ptr<Map> map;

};
