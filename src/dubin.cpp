
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

using namespace std::chrono_literals;

float dubinCurve::mod2pi(float theta) // DONE
{
  float out = theta;
  while (out < 0) out = out + 2 * M_PI;
  while (out >= 2 * M_PI) out = out - 2 * M_PI;
  return out;
};
float dubinCurve::sinc(float x)
{
  if (abs(x) < 0.002) return 1 - (x*x)/6 * (1 - (x*x)/20);
else
  return sin(x)/x;
};

geometry_msgs::msg::PoseStamped dubinCurve::pose2dToPose(pose2d pose) // DONE?
{
  geometry_msgs::msg::PoseStamped p;
  p.pose.position.x = pose.x.x;
  p.pose.position.y = pose.x.y;

  tf2::Quaternion quaternion_;
  quaternion_.setRPY(0,0,pose.theta);
  quaternion_ = quaternion_.normalize();
  p.pose.orientation.x = quaternion_.x();
  p.pose.orientation.y = quaternion_.y();
  p.pose.orientation.z = quaternion_.z();
  p.pose.orientation.w = quaternion_.w();
  return p;

};

pose2d dubinCurve::nextPointOnArc(pose2d x0, float ds, int k) // DONE
{
  pose2d next = x0;
  next.x.x += ds * sinc(k * ds / 2.0) * cos(x0.theta + k * ds / 2);
  next.x.y += ds * sinc(k * ds / 2.0) * sin(x0.theta + k * ds / 2);
  next.theta = mod2pi(next.theta + k * ds);
  return next;
};

nav_msgs::msg::Path dubinCurve::generatePathFromDubins(pose2d start, std::vector<dubins_params> sub_paths, float delta) // DONE
{
  nav_msgs::msg::Path final_path;
  final_path.header.frame_id = "map";
  pose2d currentPoint = start;
  final_path.poses.push_back(pose2dToPose(currentPoint));
  // RCLCPP_INFO(this->get_logger(),"generate path from dubins start");
  std::ofstream myfile;
  myfile.open ("example.csv");
  for (int i = 0; i < int(sub_paths.size()); i++)
  {
    for (int j = 0; j < 3; j++)
    {
      pose2d referencePoint = currentPoint;
      for (float z = 0; z < sub_paths[i].s[j]; z+=delta )
      {
        currentPoint = nextPointOnArc(referencePoint, z, sub_paths[i].k.l[j]  * this->_K);
        final_path.poses.push_back(pose2dToPose(currentPoint));
        std::string poop = std::to_string(currentPoint.x.x) + " , " +  std::to_string(currentPoint.x.y) + "\n";
        myfile << poop;          
      }
      currentPoint = nextPointOnArc(referencePoint, sub_paths[i].s[j], sub_paths[i].k.l[j]  * this->_K);
      final_path.poses.push_back(pose2dToPose(currentPoint));
      std::string poop = std::to_string(currentPoint.x.x) + " , " +  std::to_string(currentPoint.x.y) + "\n";
      myfile << poop; 
      
    }
  }
  myfile.close();
  return final_path;
};

std::vector<dubins_params> dubinCurve::calculateMultiPoint(pose2d start, pose2d end, std::vector<point2d> mid_points, int n_angles) // DONE
{
  std::vector<dubins_params> best_path;
  float angle_dif = 2* M_PI / (float)n_angles;
  pose2d x1 = end;
  pose2d x0 = start;
  for (int i = mid_points.size(); i > 0; i--)
  {
    x0.x = mid_points[i-1];
    dubins_params best_solution;
    best_solution.L = __FLT_MAX__;
    float best_theta;
    for (float theta = 0.0f; theta < 2* M_PI; theta += angle_dif)
      {
        x0.theta = theta;
        auto solution = calculateSinglePath(x0, x1);
        if (solution.L < best_solution.L)
        {
          best_solution = solution;
          best_theta = theta;
        }
        
      }
      best_path.push_back(best_solution);
      x1 = x0;
      x1.theta = best_theta;
  }
  x0 = start;
  best_path.push_back(calculateSinglePath(start, end));
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
  v.th0 = mod2pi(x0.theta - phi);
  v.th1 = mod2pi(x1.theta - phi);
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
  for (int i = 0; i < 6; i++)
  {
    path = DUBINATOR(v.th0, v.th1, v.lambda, koptions[i]);
    if ((path.L < best_path.L) && path.valid) best_path = path;
  }
  // unscale
  best_path = scaleFromStandard( v.lambda, best_path);
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
    path.s[0] = invK * mod2pi(temp1 - th0);
    float temp2 = 2 + 4 * pow(kscaled,2) - 2 * cos(th0 - th1) + 4 * kscaled * (sin(th0) - sin(th1));
    if (temp2 < 0)
    {
      path.valid = false; path.s[0] = 0; path.s[1] = 0; path.s[2] = 0;
      return path;
    }
    path.s[1] = invK * sqrt(temp2);
    path.s[2] = invK * mod2pi(th1 - temp1);
    path.valid = true;
  }
  // RSR
  else if (ks.l[0] == -1 && ks.l[1] == 0 && ks.l[2] == -1  )
  {
    float C = cos(th0) - cos(th1);
    
    float S = 2 * kscaled - sin(th0) + sin(th1);
    float temp1 = atan2(C, S);
    path.s[0] = invK * mod2pi(th0 - temp1);
    float temp2 = 2 + 4 * pow(kscaled,2) - 2 * cos(th0 - th1) - 4 * kscaled * (sin(th0) - sin(th1));
    if (temp2 < 0)
    {
      path.valid = false; path.s[0] = 0; path.s[1] = 0; path.s[2] = 0;
      return path;
    }
    path.s[1] = invK * sqrt(temp2);
    path.s[2] = invK * mod2pi(temp1 - th1);
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
    path.s[0] = invK * mod2pi(temp1 + temp2 - th0);
    path.s[2] = invK * mod2pi(temp1 + temp2 - th1);
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
    path.s[0] = invK * mod2pi(th0 - temp1 + temp2);
    path.s[2] = invK * mod2pi(th1 - temp1 + temp2);
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
    path.s[1] = invK * mod2pi(2 * M_PI - acos(temp2));
    path.s[0] = invK * mod2pi(th0 - temp1 + 0.5 * path.s[1] * kscaled);
    path.s[2] = invK * mod2pi(th0 - th1 + kscaled * (path.s[1] - path.s[0]));
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
    path.s[1] = invK * mod2pi(2 * M_PI - acos(temp2));
    path.s[0] = invK * mod2pi(temp1 - th0 + 0.5 * path.s[1] * kscaled);
    path.s[2] = invK * mod2pi(th1 - th0 + kscaled * (path.s[1] - path.s[0]));
    path.valid = true;
  }
  path.L = path.s[0] + path.s[1] + path.s[2];

  return path;
};



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

