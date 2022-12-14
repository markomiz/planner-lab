#pragma once
#include <memory>
#include <iostream>
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <math.h>
#include "tf2/exceptions.h"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"


using namespace std;
template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

///  -------------------- POINT DEFS ------------- ///
struct point2d
{
  point2d(){};
  point2d(float X, float Y): x(X), y(Y) {};
  float x;
  float y;
  float norm()
    {
        return sqrt(x*x + y*y);
    };
  point2d operator+(point2d &b)
  {
    point2d c;
    c.x = x + b.x;
    c.y = y + b.y;
    return c;
  };
  point2d operator-(point2d &b)
  {
    point2d c;
    c.x = x - b.x;
    c.y = y - b.y;
    return c;
  };
};
struct pose2d 
{
  point2d x;
  float theta;
  pose2d(){};
  pose2d(float xx,float yy,float th)
  {
    x.x = xx;
    x.y = yy;
    theta = th;
  };
  geometry_msgs::msg::PoseStamped to_Pose()
  {
    geometry_msgs::msg::PoseStamped p;
    p.pose.position.x = x.x;
    p.pose.position.y = x.y;

    tf2::Quaternion quaternion_;
    quaternion_.setRPY(0,0,theta);
    quaternion_ = quaternion_.normalize();
    p.pose.orientation.x = quaternion_.x();
    p.pose.orientation.y = quaternion_.y();
    p.pose.orientation.z = quaternion_.z();
    p.pose.orientation.w = quaternion_.w();
    return p;
  }
};

/// ------------ DUBIN RELATED STUFF --------------- ///
struct ksigns {
  int l[3];
  ksigns(int l1, int l2, int l3){

  l[0] = l1;
  l[1] = l2;
  l[2] = l3;

  };
};
struct dubins_params
{
  float s[3];
  float L;
  ksigns k = ksigns(0,0,0);
  bool valid;
  float K;

};
struct transformedVars
{
  float th0;
  float th1;
  float lambda;
};
struct line{
    point2d p_initial;
    point2d p_final;
    float length()
    {
        return sqrt((p_final.x-p_initial.x)*(p_final.x-p_initial.x)+(p_final.y-p_initial.y)*(p_final.y-p_initial.y));
    }
};
struct intersection_result{
    point2d intersection;
    bool intersects;
};
struct arc // TODO replace all arcs with this - make dubins stuff play nice with it.
{

  arc(){};
  arc(pose2d start, float K, float s) : start(start), s(s), K(K)
  {
    radius = 0;
    if (K != 0) radius = 1 / abs(K);
    int si = sgn(K);
    center.x = start.x.x + radius * cos(start.theta + si * M_PI/2);
    center.y = start.x.y + radius * sin(start.theta + si * M_PI/2);
    end = next_pose(start, s, K);
  }
  float radius;
  point2d center;
  pose2d start; 
  pose2d end;
  float K;
  float s;
  static pose2d next_pose(pose2d x0, float ds, float _k)
  {
    pose2d next = x0;
    next.x.x += ds * sinc(_k * ds / 2.0) * cos(x0.theta + _k * ds / 2);
    next.x.y += ds * sinc(_k * ds / 2.0) * sin(x0.theta + _k * ds / 2);
    next.theta = mod2pi(next.theta + _k * ds);
    return next;
  }
  static float mod2pi(float theta) // DONE
  {
    float out = theta;
    while (out < 0) out = out + 2 * M_PI;
    while (out >= 2 * M_PI) out = out - 2 * M_PI;
    return out;
  };
  static float sinc(float x)
  {
    if (abs(x) < 0.002) return 1 - (x*x)/6 * (1 - (x*x)/20);
  else
    return sin(x)/x;
  };
};
struct arcs
{
  arcs(){};
  arcs(pose2d start, dubins_params dp) {
    a[0] = arc(start, dp.K * dp.k.l[0] , dp.s[0]);
    a[1] = arc(a[0].end, dp.K * dp.k.l[1] , dp.s[1]);
    a[2] = arc(a[1].end, dp.K * dp.k.l[2] , dp.s[2]);
    L = dp.L;
  };
  
  arc a[3];
  float L;
  arcs get_inverse(){
    arcs ARCS = *this;
    ARCS.a[0].start = this->a[2].end;
    ARCS.a[0].start.theta = arc::mod2pi(this->a[2].end.theta + M_PI);
    ARCS.a[0].K = - this->a[2].K;
    ARCS.a[0].end = this->a[2].start;
    ARCS.a[0].end.theta = arc::mod2pi(this->a[2].start.theta + M_PI);

    ARCS.a[1].start = this->a[1].end;
    ARCS.a[1].start.theta = arc::mod2pi(this->a[1].end.theta + M_PI);
    ARCS.a[1].K = - this->a[1].K;
    ARCS.a[1].end = this->a[1].start;
    ARCS.a[1].end.theta = arc::mod2pi(this->a[1].start.theta + M_PI);

    ARCS.a[2].start = this->a[0].end;
    ARCS.a[2].start.theta = arc::mod2pi(this->a[0].end.theta + M_PI);
    ARCS.a[2].K = - this->a[0].K;
    ARCS.a[2].end = this->a[0].start;
    ARCS.a[2].end.theta = arc::mod2pi(this->a[0].start.theta + M_PI);
    
    return ARCS;

  }

};
