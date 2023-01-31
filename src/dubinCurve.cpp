
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

#include "MissionPlanner.h"
#include "Planner.h"
#include "dubinCurve.h"

using namespace std::chrono_literals;

nav_msgs::msg::Path dubinCurve::generatePathFromDubins(pose2d start, std::vector<dubins_params> sub_paths, float delta) // DONE
{
  nav_msgs::msg::Path final_path;
  final_path.header.frame_id = "map";
  pose2d currentPoint = start;
  final_path.poses.push_back(currentPoint.to_Pose());
  for (auto i = 0; i < int(sub_paths.size()); i++)
  {
    for (auto j = 0; j < 3; j++)
    {
      pose2d referencePoint = currentPoint;
      for (float z = 0; z < sub_paths[i].s[j]; z+=delta )
      {
        currentPoint = arc::next_pose(referencePoint, z, sub_paths[i].k.l[j] * _K );
        final_path.poses.push_back(currentPoint.to_Pose());         
      }
      currentPoint = arc::next_pose(referencePoint, sub_paths[i].s[j], sub_paths[i].k.l[j] * _K );
      final_path.poses.push_back(currentPoint.to_Pose());
    }
  }
  return final_path;
};
nav_msgs::msg::Path dubinCurve::arcs_to_path(deque<arcs> input_arcs, float delta)
{
  nav_msgs::msg::Path final_path;
  final_path.header.frame_id = "map";

  pose2d currentPoint = input_arcs[0].a[0].start;

  final_path.poses.push_back(currentPoint.to_Pose());
  for (auto i = 0; i < int(input_arcs.size()); i++)
  {
    currentPoint =  input_arcs[i].a[0].start;
    for (auto j = 0; j < 3; j++)
    {
      arc &a = input_arcs[i].a[j];
      
      for (float ds = 0; ds < a.s; ds += delta)
      {
        currentPoint = arc::next_pose(a.start, ds, a.K);
        final_path.poses.push_back(currentPoint.to_Pose());
      }
      currentPoint = a.end;
      final_path.poses.push_back(currentPoint.to_Pose());
    }
  }
  return final_path;
}
std::deque<arcs> dubinCurve::calculateMultiPoint(pose2d start, pose2d end, std::deque<point2d> mid_points, int n_angles) // DONE
{
  std::deque<arcs> best_path;
  float angle_dif = 2* M_PI / (float(n_angles));
  pose2d x1 = end;
  pose2d x0 = start;
  cout << angle_dif << " angle dif" <<endl;
  for (int i = mid_points.size(); i > 0; i--)
  {
    
    x0.x = mid_points[i-1];
    arcs best_solution;
    best_solution.L = __FLT_MAX__;
    float best_theta;
    for (float theta = 0.0f; theta < 2* M_PI; theta += angle_dif)
      {
        // cout << theta << " theta " << i << endl;
        x0.theta = theta;
        
        auto solution = calculateSinglePath(x0, x1);
        arcs A = arcs(x0, solution);
        if (map->colliding(A) ) continue;
        if (A.L < best_solution.L )
        {
          best_solution = A;
          best_theta = theta;
        }
      }
      if (best_solution.L == __FLT_MAX__)
      {
        cout << "NO WAY!";
        std::deque<arcs> empty;
        return empty;
      }

      best_path.push_front(best_solution);
      x1 = x0;
      x1.theta = best_theta;
  }
  x0 = start;
  auto solution = calculateSinglePath(x0, x1);
  arcs A = arcs(x0, solution);
  best_path.push_front(A);
  return best_path;
};
// Scale the input problem to standard form (x0: -1, y0: 0, xf: 1, yf: 0)
transformedVars dubinCurve::scaleToStandard(pose2d x0, pose2d x1)
{
  // RCLCPP_INFO(this->get_logger(),"scale");
  transformedVars v;
    // find transform parameters
  float dx = x1.x.x - x0.x.x;
  float dy = x1.x.y - x0.x.y;
  float phi = atan2(dy, dx);
  v.lambda = hypot(dx, dy)/2; 

  // scale and normalize angles and curvature
  v.th0 = arc::mod2pi(x0.theta - phi);
  v.th1 = arc::mod2pi(x1.theta - phi);
  return v;
};

// Scale the solution to the standard problem back to the original problem
dubins_params dubinCurve::scaleFromStandard(float lambda, dubins_params params)
{
  dubins_params rescaled = params;
  rescaled.s[0] *= lambda;
  rescaled.s[1] *= lambda;
  rescaled.s[2] *= lambda;
  rescaled.L *= lambda;
  return rescaled;
};

dubins_params dubinCurve::calculateSinglePath( pose2d x0, pose2d x1 ){
  dubins_params best_path;
  best_path.L = __FLT_MAX__;
  dubins_params path;
  std::vector<ksigns> koptions;
  koptions.push_back(ksigns(1,0,1));
  koptions.push_back(ksigns(-1,0,-1));
  koptions.push_back(ksigns(1,0,-1));
  koptions.push_back(ksigns(-1,0,1));
  koptions.push_back(ksigns(1,-1,1));
  koptions.push_back(ksigns(-1,1,-1));
  // go through cases 
  // RCLCPP_INFO(this->get_logger(),"calc single");
  // scale
  transformedVars v = scaleToStandard(x0,x1);
  for (auto i = 0; i < 6; i++)
  {
    path = DUBINATOR(v.th0, v.th1, v.lambda, koptions[i]);
    if ((path.L < best_path.L) && path.valid) best_path = path;
  }
  // unscale
  best_path = scaleFromStandard( v.lambda, best_path);
  best_path.K = this->_K;
  return best_path;
};

dubins_params dubinCurve::DUBINATOR( float th0, float th1, float lambda, ksigns ks ) // DONE
{
  dubins_params path;
  path.k = ks;
  // RCLCPP_INFO(this->get_logger(),"dub start");
  float kscaled = lambda * this->_K;
  
    // LSL
  float invK = 1 / kscaled;
  if (ks.l[0] == 1 && ks.l[1] == 0 && ks.l[2] == 1  )
  {
    float C = cos(th1) - cos(th0);
    float S = 2 * kscaled + sin(th0) - sin(th1);
    float temp1 = atan2(C, S);
    path.s[0] = invK * arc::mod2pi(temp1 - th0);
    float temp2 = 2 + 4 * pow(kscaled,2) - 2 * cos(th0 - th1) + 4 * kscaled * (sin(th0) - sin(th1));
    if (temp2 < 0)
    {
      path.valid = false; path.s[0] = 0; path.s[1] = 0; path.s[2] = 0;
      return path;
    }
    path.s[1] = invK * sqrt(temp2);
    path.s[2] = invK * arc::mod2pi(th1 - temp1);
    path.valid = true;
  }
  // RSR
  else if (ks.l[0] == -1 && ks.l[1] == 0 && ks.l[2] == -1  )
  {
    float C = cos(th0) - cos(th1);
    
    float S = 2 * kscaled - sin(th0) + sin(th1);
    float temp1 = atan2(C, S);
    path.s[0] = invK * arc::mod2pi(th0 - temp1);
    float temp2 = 2 + 4 * pow(kscaled,2) - 2 * cos(th0 - th1) - 4 * kscaled * (sin(th0) - sin(th1));
    if (temp2 < 0)
    {
      path.valid = false; path.s[0] = 0; path.s[1] = 0; path.s[2] = 0;
      return path;
    }
    path.s[1] = invK * sqrt(temp2);
    path.s[2] = invK * arc::mod2pi(temp1 - th1);
    path.valid = true;
  }
  else if (ks.l[0] == 1 && ks.l[1] == 0 && ks.l[2] == -1  )
  {
  // LSR

    float C = cos(th0) + cos(th1);
    
    float S = 2 * kscaled + sin(th0) + sin(th1);
    float temp1 = atan2(-C, S);
    float temp3 = 4 * pow(kscaled,2) - 2 + 2 * cos(th0 - th1) + 4 * kscaled * (sin(th0) + sin(th1));
    if (temp3 < 0)
    {
      path.valid = false; path.s[0] = 0; path.s[1] = 0; path.s[2] = 0;
      return path;
    }
      path.s[1] = invK * sqrt(temp3);
    float temp2 = -atan2(-2, path.s[1] * kscaled);
    path.s[0] = invK * arc::mod2pi(temp1 + temp2 - th0);
    path.s[2] = invK * arc::mod2pi(temp1 + temp2 - th1);
    path.valid = true;
  }
  else if (ks.l[0] == -1 && ks.l[1] == 0 && ks.l[2] == 1  )
  {
  // RSL

    float C = cos(th0) + cos(th1);
    
    float S = 2 * kscaled - sin(th0) - sin(th1);
    float temp1 = atan2(C, S);
    float temp3 = 4 * pow(kscaled,2) - 2 + 2 * cos(th0 - th1) - 4 * kscaled * (sin(th0) + sin(th1));
    if (temp3 < 0)
    {
      path.valid = false; path.s[0] = 0; path.s[1] = 0; path.s[2] = 0;
      return path;
    }
    path.s[1] = invK * sqrt(temp3);
    float temp2 = atan2(2, path.s[1] * kscaled);
    path.s[0] = invK * arc::mod2pi(th0 - temp1 + temp2);
    path.s[2] = invK * arc::mod2pi(th1 - temp1 + temp2);
    path.valid = true;
  }
  else if (ks.l[0] == -1 && ks.l[1] == 1 && ks.l[2] == -1  )
  {
  // RLR

    float C = cos(th0) - cos(th1);
    
    float S = 2 * kscaled - sin(th0) + sin(th1);
    float temp1 = atan2(C, S);
    float temp2 = 0.125 * (6 - 4 * pow(kscaled,2) + 2 * cos(th0 - th1) + 4 * kscaled * (sin(th0) - sin(th1)));
    if (abs(temp2) > 1)
    {
      path.valid = false; path.s[0] = 0; path.s[1] = 0; path.s[2] = 0;
      return path;
    }
    path.s[1] = invK * arc::mod2pi(2 * M_PI - acos(temp2));
    path.s[0] = invK * arc::mod2pi(th0 - temp1 + 0.5 * path.s[1] * kscaled);
    path.s[2] = invK * arc::mod2pi(th0 - th1 + kscaled * (path.s[1] - path.s[0]));
    path.valid = true;
  }
  else if (ks.l[0] == -1 && ks.l[1] == 1 && ks.l[2] == -1  )
  {
// LRL

    float C = cos(th1) - cos(th0);
    
    float S = 2 * kscaled + sin(th0) - sin(th1);
    float temp1 = atan2(C, S);
    float temp2 = 0.125 * (6 - 4 * pow(kscaled,2) + 2 * cos(th0 - th1) - 4 * kscaled * (sin(th0) - sin(th1)));
    if (abs(temp2) > 1)
    {
      path.valid = false; path.s[0] = 0; path.s[1] = 0; path.s[2] = 0;
      return path;
    }
    path.s[1] = invK * arc::mod2pi(2 * M_PI - acos(temp2));
    path.s[0] = invK * arc::mod2pi(temp1 - th0 + 0.5 * path.s[1] * kscaled);
    path.s[2] = invK * arc::mod2pi(th1 - th0 + kscaled * (path.s[1] - path.s[0]));
    path.valid = true;
  }
  path.L = path.s[0] + path.s[1] + path.s[2];

  return path;
};

