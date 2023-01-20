#pragma once
#include <chrono>
#include <memory>
#include <vector>
#include <math.h>
#include <iostream>
#include <fstream>
#include <unistd.h>
#include <string>
#include <deque>

#include "geometry.h"
#include "graph.h"
#include "map.h"
#include "PRMstar.h"
#include "dubinCurve.h"
#include "config_server.h"
#include "MissionPlanner.h"

#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "obstacles_msgs/msg/obstacle_array_msg.hpp"
#include "obstacles_msgs/msg/obstacle_msg.hpp"
#include "std_msgs/msg/header.hpp"

#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;

void MissionPlanner::obstacle_topic_callback(const obstacles_msgs::msg::ObstacleArrayMsg obstacle_message)
{
    if(is_receive_obs)
    {
        return;
    }
    // create polygon obstacle object
    int n_obs = obstacle_message.obstacles.size();
    
    for (int i = 0; i < n_obs; i++)
    {
        vector<point2d> polygon_input = {};
        // RCLCPP_INFO(this->get_logger(), "Obstacle '%i' points:", i);
        geometry_msgs::msg::Polygon aux = obstacle_message.obstacles[i].polygon;
        int nr_points = aux.points.size();
        for (int j = 0; j < nr_points; j++)
        {
            point2d temp;
            temp.x = float(aux.points[j].x);
            temp.y = float(aux.points[j].y);
            polygon_input.push_back(temp);
            // RCLCPP_INFO(this->get_logger(), "Getting obs info: x = '%0.2f', y = '%0.2f'", temp.x, temp.y);
        }
        Polygon poly(polygon_input);
        obstacle_list.push_back(poly);
    }
    RCLCPP_INFO(this->get_logger(), "Got obstacles information");
    is_receive_obs = true;
    getPaths_and_Publish();

};

void MissionPlanner::map_topic_callback(const geometry_msgs::msg::PolygonStamped outline_message)
{
    if(is_receive_map)
    {
        return;
    }
    vector<point2d> outer_verteces;
    for (int i = 0; i < outline_message.polygon.points.size(); i++)
    {
        point2d temp;
        temp.x = outline_message.polygon.points[i].x;
        temp.y = outline_message.polygon.points[i].y;
        outer_verteces.push_back(temp);        
    }
    map_poly = Polygon(outer_verteces);
    is_receive_map = true;
    RCLCPP_INFO(this->get_logger(), "Got map information.");
    getPaths_and_Publish();
};

void MissionPlanner::gate_topic_callback(const geometry_msgs::msg::PoseArray outline_message)
{
    if(is_receive_gate)
    {
        return;
    }
    cout << "the gates are " << outline_message.poses.size() << endl;
    for (int i = 0; i < outline_message.poses.size(); i++)
    {
        pose2d temp_gate;
        tf2::Quaternion q(outline_message.poses[i].orientation.x, outline_message.poses[i].orientation.y, outline_message.poses[i].orientation.z, outline_message.poses[i].orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        temp_gate.x.x  = outline_message.poses[i].position.x;
        temp_gate.x.y  = outline_message.poses[i].position.y;
        temp_gate.theta = yaw;
        gate.push_back(temp_gate);   
    }
    is_receive_gate = true;
    RCLCPP_INFO(this->get_logger(), "Got gates information");
    getPaths_and_Publish();
    
};

void MissionPlanner::pose_topic_callback(const geometry_msgs::msg::TransformStamped t)
{
    if(poses_received == 1)
    {
        return;
    }
    pose2d temp_init_pose(__FLT_MAX__,__FLT_MAX__,__FLT_MAX__);
    tf2::Quaternion q(t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    temp_init_pose.x.x  = t.transform.translation.x;
    temp_init_pose.x.y  = t.transform.translation.y;
    temp_init_pose.theta = yaw;
    initial_pose.push_back(temp_init_pose);
    poses_received++;
    RCLCPP_INFO(this->get_logger(), "Got initial poses");
    getPaths_and_Publish();
};

void MissionPlanner::build_roadmap()
{
    // shared_ptr<Map> map (new Map(map_poly));
    point2d t1(-8,-8);
    point2d t2(-8,8);
    point2d t3(8,8);
    point2d t4(8,-8);

    point2d p1(1,4);
    point2d p2(2,5);
    point2d p3(5,2);
    point2d p4(4,1);
    
    vector<point2d> vec_vert;
    vec_vert.push_back(t1);
    vec_vert.push_back(t2);
    vec_vert.push_back(t3);
    vec_vert.push_back(t4);

    // vector<point2d> obs_vert;
    // obs_vert.push_back(p1);
    // obs_vert.push_back(p2);
    // obs_vert.push_back(p3);
    // obs_vert.push_back(p4);

    
    Polygon test_map(vec_vert); 

    shared_ptr<Map> map (new Map(test_map));

    

    for (int i = 0; i < obstacle_list.size(); i++)
    {
        map->addObstacle(obstacle_list[i]);
    }

    RCLCPP_INFO(this->get_logger(),"Map made and Obstacles included. Free space = %0.2f", map->getFreeSpace());
    
    shared_ptr<dubinCurve> d (new dubinCurve())
    d->map = map;
    d->_K = conf->getK();

    planner = new PRMstar(map);
    RCLCPP_INFO(this->get_logger(),"Planner made");
    planner->dCurve = d;
    planner->genRoadmapPlus(conf->getNumPoints(), conf->getNumAngles());
    RCLCPP_INFO(this->get_logger(),"Roadmap Generated");

};

void MissionPlanner::publish_results(int robot)
{
    /*
        ACTION CLIENT AND SERVER PART
    */
    string robot_str = "shelfino" + std::to_string(robot) + "/follow_path";
    publisher_ = this->create_publisher<nav_msgs::msg::Path>(robot_str, 10);

    using FollowPath = nav2_msgs::action::FollowPath;

    rclcpp_action::Client<FollowPath>::SharedPtr client_ptr_;
    client_ptr_ = rclcpp_action::create_client<FollowPath>(this,robot_str);

    if (!client_ptr_->wait_for_action_server()) {
        cout << "here!!!" << endl;
        RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
        rclcpp::shutdown();
    }
    
    auto goal_msg = FollowPath::Goal();
    goal_msg.path = path;
    goal_msg.controller_id = "FollowPath";
    
    RCLCPP_INFO(this->get_logger(), "Sending goal position");
    client_ptr_->async_send_goal(goal_msg);

    for(int i = 0; i<2; i++)
    {
        publisher_->publish(path);
        usleep(1000000);
        RCLCPP_INFO(this->get_logger(), "%s", path.header.frame_id.c_str());
    }
};

void MissionPlanner::getPaths_and_Publish()
{
    if (is_receive_map && is_receive_gate && is_receive_obs && poses_received == 2)
    {
        RCLCPP_INFO(this->get_logger(), "Calculating Roadmap");
        build_roadmap();
        cout << "I got " <<initial_pose.size() << " initial poses" << endl;
        for (int i = 0; i < initial_pose.size(); i++)
        {
            pose2d xf(5,5,0.5);
            deque<arcs> way = planner->getPath(initial_pose[i],xf);
            path = d->arcs_to_path(way, 0.05);
            RCLCPP_INFO(this->get_logger(),"Path found");

            // Publish results
            RCLCPP_INFO(this->get_logger(), "Publishing path");
            publish_results(i+1);
            //do some computations and publish messages
        }
        rclcpp::shutdown();
    }
};



int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MissionPlanner>());
  rclcpp::shutdown();
  return 0;
};