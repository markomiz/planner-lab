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
#include <ctime>

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
    // getPaths_and_Publish();

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
    // getPaths_and_Publish();
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
    // getPaths_and_Publish();
    
};

void MissionPlanner::pose1_topic_callback(const geometry_msgs::msg::TransformStamped t)
{
    if(is_receive_pose1)
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
    is_receive_pose1 = true;
    RCLCPP_INFO(this->get_logger(), "Got initial pose 1");
    // getPaths_and_Publish();
};

void MissionPlanner::pose2_topic_callback(const geometry_msgs::msg::TransformStamped t)
{
    if(is_receive_pose2 && is_receive_pose1)
    {
        getPaths_and_Publish();
        return;
    } else if (is_receive_pose1)
    {
        pose2d temp_init_pose(__FLT_MAX__,__FLT_MAX__,__FLT_MAX__);
        tf2::Quaternion q(t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        temp_init_pose.x.x  = t.transform.translation.x;
        temp_init_pose.x.y  = t.transform.translation.y;
        temp_init_pose.theta = yaw;
        initial_pose.push_back(temp_init_pose);
        is_receive_pose2 = true;
        RCLCPP_INFO(this->get_logger(), "Got initial pose 2");
    } else return;
};

void MissionPlanner::build_roadmap()
{
    // shared_ptr<Map> map (new Map(map_poly));
    point2d t1(-8,-8);
    point2d t2(-8,8);
    point2d t3(8,8);
    point2d t4(8,-8);
    
    vector<point2d> vec_vert;
    vec_vert.push_back(t1);
    vec_vert.push_back(t2);
    vec_vert.push_back(t3);
    vec_vert.push_back(t4);

    Polygon test_map(vec_vert); 

    shared_ptr<Map> map (new Map(test_map));
    for (int i = 0; i < obstacle_list.size(); i++)
    {
        map->addObstacle(obstacle_list[i]);
    }

    RCLCPP_INFO(this->get_logger(),"Map made and Obstacles included. Free space = %0.2f", map->getFreeSpace());

    d->map = map;
    d->_K = conf->getK();

    planner = new PRMstar(map);
    planner->config = conf;
    planner->graph->config = conf;
    RCLCPP_INFO(this->get_logger(),"Planner made");
    planner->dCurve = d;
    planner->genRoadmapPlus(conf->getNumPoints(), conf->getNumAngles());
    RCLCPP_INFO(this->get_logger(),"Roadmap Generated");

};

// void MissionPlanner::publish_results(int robot)
// {
//     /*
//         ACTION CLIENT AND SERVER PART
//     */
//     string robot_str = "shelfino" + std::to_string(robot) + "/follow_path";
//     publisher_ = this->create_publisher<nav_msgs::msg::Path>(robot_str, 10);
//     rclcpp_action::Client<FollowPath>::SharedPtr client_ptr_;
//     client_ptr_ = rclcpp_action::create_client<FollowPath>(this,robot_str);
//     if (!client_ptr_->wait_for_action_server()) {
//         cout << "here!!!" << endl;
//         RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
//         rclcpp::shutdown();
//     }    
//     auto goal_msg = FollowPath::Goal();
//     goal_msg.path = path;
//     goal_msg.controller_id = "FollowPath";   
//     RCLCPP_INFO(this->get_logger(), "Sending goal position");
//     client_ptr_->async_send_goal(goal_msg);
//     for(int i = 0; i<2; i++)
//     {
//         publisher_->publish(path);
//         usleep(1000000);
//         RCLCPP_INFO(this->get_logger(), "%s", path.header.frame_id.c_str());
//     }
// };

void MissionPlanner::getPaths_and_Publish()
{
    if(is_receive_map && is_receive_gate && is_receive_obs && is_receive_pose1 && is_receive_pose2)
    {
        RCLCPP_INFO(this->get_logger(), "Got all information from simulation");
        RCLCPP_INFO(this->get_logger(), "Calculating Roadmap");
        clock_t beforeTime = clock();
        build_roadmap();
        clock_t afterTime = clock() - beforeTime;
        
        cout << "Building the roadmap took " <<(float)afterTime/CLOCKS_PER_SEC << " seconds." << endl;
        
        pathFinder1();
        pathFinder2();

        rclcpp::shutdown();
    }
};

void MissionPlanner::map_subscriber()
{
    RCLCPP_INFO(this->get_logger(), "Getting map info");
    rclcpp::QoS qos_map = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);
    map_subscription_ = this->create_subscription<geometry_msgs::msg::PolygonStamped>(
    "map_borders", qos_map, std::bind(&MissionPlanner::map_topic_callback, this, _1));
};

void MissionPlanner::obs_subscriber()
{
    RCLCPP_INFO(this->get_logger(), "Getting obstacle info");
    rclcpp::QoS qos_obs = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);
    obs_subscription_ = this->create_subscription<obstacles_msgs::msg::ObstacleArrayMsg>(
    "obstacles", qos_obs, std::bind(&MissionPlanner::obstacle_topic_callback, this, _1));
};

void MissionPlanner::gate_subscriber()
{
    RCLCPP_INFO(this->get_logger(), "Getting gate info");
    rclcpp::QoS qos_gate = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);
    gate_subscription_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
    "gate_position", qos_gate, std::bind(&MissionPlanner::gate_topic_callback, this, _1));
};

void MissionPlanner::pose1_subscriber()
{
    string name1 = "shelfino1/transform"; 
    RCLCPP_INFO(this->get_logger(), "Getting position info for shelfino 1");
    rclcpp::QoS qos_pose = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);

    pose1_subscription_ = this->create_subscription<geometry_msgs::msg::TransformStamped>(
    name1, qos_pose, std::bind(&MissionPlanner::pose1_topic_callback, this, _1));
};

void MissionPlanner::pose2_subscriber()
{
    string name2 = "shelfino2/transform"; 
    RCLCPP_INFO(this->get_logger(), "Getting position info for shelfino 2");
    rclcpp::QoS qos_pose = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);
    pose2_subscription_ = this->create_subscription<geometry_msgs::msg::TransformStamped>(
    name2, qos_pose, std::bind(&MissionPlanner::pose2_topic_callback, this, _1));
};

void MissionPlanner::pathFinder1()
{
    pose2d x0(5,5,0.5);
    deque<arcs> way = planner->getPath(initial_pose[0], x0);
    path1 = d->arcs_to_path(way, 0.05);
    RCLCPP_INFO(this->get_logger(),"Path1 found");

    // Publish results
    RCLCPP_INFO(this->get_logger(), "Publishing path");
    
    /*
        ACTION CLIENT AND SERVER PART
    */
    string robot_str = "shelfino1/follow_path";
    publisher_ = this->create_publisher<nav_msgs::msg::Path>(robot_str, 10);
    
    client_ptr_1 = rclcpp_action::create_client<FollowPath>(this,robot_str);

    if (!client_ptr_1->wait_for_action_server()) {
        cout << "here!!!" << endl;
        RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
        rclcpp::shutdown();
    }
    
    auto goal_msg1 = FollowPath::Goal();
    goal_msg1.path = path1;
    goal_msg1.controller_id = "FollowPath";
    
    RCLCPP_INFO(this->get_logger(), "Sending goal1 position");
    client_ptr_1->async_send_goal(goal_msg1);

    for(int i = 0; i<2; i++)
    {
        publisher_->publish(path1);
        usleep(1000000);
        RCLCPP_INFO(this->get_logger(), "%s", path1.header.frame_id.c_str());
    }
};

void MissionPlanner::pathFinder2()
{
    pose2d x0(5,5,0.5);
    deque<arcs> way = planner->getPath(initial_pose[1],x0);
    path2 = d->arcs_to_path(way, 0.05);
    RCLCPP_INFO(this->get_logger(),"Path2 found");

    // Publish results
    RCLCPP_INFO(this->get_logger(), "Publishing path");
    /*
        ACTION CLIENT AND SERVER PART
    */
    string robot_str = "shelfino2/follow_path";
    publisher_ = this->create_publisher<nav_msgs::msg::Path>(robot_str, 10);
    
    client_ptr_2 = rclcpp_action::create_client<FollowPath>(this,robot_str);

    if (!client_ptr_2->wait_for_action_server()) {
        cout << "here!!!" << endl;
        RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
        rclcpp::shutdown();
    }

    
    auto goal_msg2 = FollowPath::Goal();
    goal_msg2.path = path2;
    goal_msg2.controller_id = "FollowPath";
    
    RCLCPP_INFO(this->get_logger(), "Sending goal2 position");
    client_ptr_2->async_send_goal(goal_msg2);

    for(int i = 0; i<2; i++)
    {
        publisher_->publish(path2);
        usleep(1000000);
        RCLCPP_INFO(this->get_logger(), "%s", path2.header.frame_id.c_str());
    }

};

void MissionPlanner::subscribe_to_map_info()
{
    RCLCPP_INFO(this->get_logger(), "Getting map info");
    rclcpp::QoS qos_map = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);
    map_subscription_ = this->create_subscription<geometry_msgs::msg::PolygonStamped>(
    "map_borders", qos_map, std::bind(&MissionPlanner::map_topic_callback, this, _1));
};
void MissionPlanner::subscribe_to_obstacle_info()
{
    // Create subscribers for gate, map and obstacles
    RCLCPP_INFO(this->get_logger(), "Getting obstacle info");
    rclcpp::QoS qos_obs = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);
    obs_subscription_ = this->create_subscription<obstacles_msgs::msg::ObstacleArrayMsg>(
    "obstacles", qos_obs, std::bind(&MissionPlanner::obstacle_topic_callback, this, _1));

};
void MissionPlanner::subscribe_to_gate_info()
{
    RCLCPP_INFO(this->get_logger(), "Getting gate info");
    rclcpp::QoS qos_gate = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);
    gate_subscription_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
    "gate_position", qos_gate, std::bind(&MissionPlanner::gate_topic_callback, this, _1));
};
void MissionPlanner::subscribe_to_shelfino1()
{
    // Define robot currently working
    name = "shelfino2/transform"; 
    // RCLCPP_INFO(this->get_logger(), "%s", name.c_str());
    
    // get initial pose
    RCLCPP_INFO(this->get_logger(), "Getting position info for shelfino %i", 1);
    rclcpp::QoS qos_pose = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);

    pose_subscription_[1] = this->create_subscription<geometry_msgs::msg::TransformStamped>(
    name, qos_pose, std::bind(&MissionPlanner::pose_topic_callback, this, _1));
};
void MissionPlanner::subscribe_to_shelfino2()
{
    // Define robot currently working
    name = "shelfino2/transform"; 
    // RCLCPP_INFO(this->get_logger(), "%s", name.c_str());
    
    // get initial pose
    RCLCPP_INFO(this->get_logger(), "Getting position info for shelfino %i", 2);
    rclcpp::QoS qos_pose = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);

    pose_subscription_[2] = this->create_subscription<geometry_msgs::msg::TransformStamped>(
    name, qos_pose, std::bind(&MissionPlanner::pose_topic_callback, this, _1));
};
void MissionPlanner::subscribe_to_shelfino3()
{
    // Define robot currently working
    name = "shelfino3/transform"; 
    // RCLCPP_INFO(this->get_logger(), "%s", name.c_str());
    
    // get initial pose
    RCLCPP_INFO(this->get_logger(), "Getting position info for shelfino %i", 3);
    rclcpp::QoS qos_pose = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);

    pose_subscription_[3] = this->create_subscription<geometry_msgs::msg::TransformStamped>(
    name, qos_pose, std::bind(&MissionPlanner::pose_topic_callback, this, _1));
};

void MissionPlanner::find_best_paths_combo(){

    for (int i = 0; i < 9; i++)
    {
        // first bit is shelfino1 exit

        // second bit is shelfino2 exit

        // third bit is shelfino3 exit
    }

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MissionPlanner>());
  rclcpp::shutdown();
  return 0;
};