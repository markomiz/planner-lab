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
#include "Planner.h"
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

/*
----- Subscribers and Publishers -----------
*/

void MissionPlanner::subscribe_to_map()
{
    RCLCPP_INFO(this->get_logger(), "Getting map info");
    rclcpp::QoS qos_map = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);
    map_subscription_ = this->create_subscription<geometry_msgs::msg::Polygon>(
    "map_borders", qos_map, std::bind(&MissionPlanner::map_topic_callback, this, _1));
};

void MissionPlanner::subscribe_to_obstacles()
{
    RCLCPP_INFO(this->get_logger(), "Getting obstacle info");
    rclcpp::QoS qos_obs = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);
    obs_subscription_ = this->create_subscription<obstacles_msgs::msg::ObstacleArrayMsg>(
    "obstacles", qos_obs, std::bind(&MissionPlanner::obstacle_topic_callback, this, _1));
};

void MissionPlanner::subscribe_to_gate()
{
    RCLCPP_INFO(this->get_logger(), "Getting gate info");
    rclcpp::QoS qos_gate = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);
    gate_subscription_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
    "gate_position", qos_gate, std::bind(&MissionPlanner::gate_topic_callback, this, _1));
};

void MissionPlanner::subscribe_to_pose1()
{
    string name1 = "shelfino1/transform"; 
    RCLCPP_INFO(this->get_logger(), "Getting position info for shelfino 1");
    rclcpp::QoS qos_pose = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);

    pose1_subscription_ = this->create_subscription<geometry_msgs::msg::TransformStamped>(
    name1, qos_pose, std::bind(&MissionPlanner::pose1_topic_callback, this, _1));
};

void MissionPlanner::subscribe_to_pose2()
{
    string name2 = "shelfino2/transform"; 
    RCLCPP_INFO(this->get_logger(), "Getting position info for shelfino 2");
    rclcpp::QoS qos_pose = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);
    pose2_subscription_ = this->create_subscription<geometry_msgs::msg::TransformStamped>(
    name2, qos_pose, std::bind(&MissionPlanner::pose2_topic_callback, this, _1));
};

void MissionPlanner::publish_path(string topic, deque<arcs> way)
{

    nav_msgs::msg::Path path = d->arcs_to_path(way, 0.05);
    RCLCPP_INFO(this->get_logger(),"Path %s found", topic.c_str());
    // Publish results
    RCLCPP_INFO(this->get_logger(), "Publishing path");
    /*
        ACTION CLIENT AND SERVER PART
    */

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher = this->create_publisher<nav_msgs::msg::Path>(topic, 10);
    rclcpp_action::Client<FollowPath>::SharedPtr client_ptr = rclcpp_action::create_client<FollowPath>(this,topic);

    if (!client_ptr->wait_for_action_server()) {
        RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
        rclcpp::shutdown();
    }
    
    auto goal_msg2 = FollowPath::Goal();
    goal_msg2.path = path;
    goal_msg2.controller_id = "FollowPath";
    
    RCLCPP_INFO(this->get_logger(), "Sending goal position: ");
    client_ptr->async_send_goal(goal_msg2);

    for(auto i = 0; i<2; i++)
    {
        publisher->publish(path);
        usleep(1000000);
        RCLCPP_INFO(this->get_logger(), "%s", path.header.frame_id.c_str());
    }
}

/*
----- Callbacks for subscribers -----------
*/

void MissionPlanner::obstacle_topic_callback(const obstacles_msgs::msg::ObstacleArrayMsg obstacle_message)
{
    if(has_received_obs) return;
    
    // create polygon obstacle object
    int n_obs = obstacle_message.obstacles.size();
    
    for (auto i = 0; i < n_obs; i++)
    {
        vector<point2d> polygon_input = {};
        // RCLCPP_INFO(this->get_logger(), "Obstacle '%i' points:", i);
        geometry_msgs::msg::Polygon aux = obstacle_message.obstacles[i].polygon;
        int nr_points = aux.points.size();
        for (auto j = 0; j < nr_points; j++)
        {
            point2d temp;
            temp.x = float(aux.points[j].x);
            temp.y = float(aux.points[j].y);
            polygon_input.push_back(temp);
            // RCLCPP_INFO(this->get_logger(), "Getting obs info: x = '%0.2f', y = '%0.2f'", temp.x, temp.y);
        }
        Polygon poly(polygon_input, conf->getExpandSize());
        // poly.expandShape(conf->getExpandSize());
        obstacle_list.push_back(poly);
    }
    RCLCPP_INFO(this->get_logger(), "Got obstacles information");
    has_received_obs = true;
    getPaths_and_Publish();

};

void MissionPlanner::map_topic_callback(const geometry_msgs::msg::Polygon outline_message)
{
    if(has_received_map) return;
    
    vector<point2d> outer_verteces;
    for (auto i = 0; i < outline_message.points.size(); i++)
    {
        point2d temp;
        temp.x = outline_message.points[i].x;
        temp.y = outline_message.points[i].y;
        outer_verteces.push_back(temp);        
    }
    map_poly = Polygon(outer_verteces, conf->getExpandSize());
    has_received_map = true;
    RCLCPP_INFO(this->get_logger(), "Got map information.");
    getPaths_and_Publish();
};

void MissionPlanner::gate_topic_callback(const geometry_msgs::msg::PoseArray outline_message)
{
    if(has_received_gate) return;
    
    cout << "the gates are " << outline_message.poses.size() << endl;
    for (auto i = 0; i < outline_message.poses.size(); i++)
    {
        pose2d temp_gate;
        tf2::Quaternion q(outline_message.poses[i].orientation.x, outline_message.poses[i].orientation.y, outline_message.poses[i].orientation.z, outline_message.poses[i].orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        temp_gate.x.x  = outline_message.poses[i].position.x;
        temp_gate.x.y  = outline_message.poses[i].position.y;
        temp_gate.theta = 0;
        gates.push_back(temp_gate);   
    }
    has_received_gate = true;
    RCLCPP_INFO(this->get_logger(), "Got gates information");
    getPaths_and_Publish();
    
};

void MissionPlanner::pose1_topic_callback(const geometry_msgs::msg::TransformStamped t)
{
    if(has_received_pose1) return;
    
    pose2d temp_init_pose(__FLT_MAX__,__FLT_MAX__,__FLT_MAX__);
    tf2::Quaternion q(t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    temp_init_pose.x.x  = t.transform.translation.x;
    temp_init_pose.x.y  = t.transform.translation.y;
    temp_init_pose.theta = yaw;
    initial_poses.push_back(temp_init_pose);
    has_received_pose1 = true;
    RCLCPP_INFO(this->get_logger(), "Got initial pose 1");
    getPaths_and_Publish();
};

void MissionPlanner::pose2_topic_callback(const geometry_msgs::msg::TransformStamped t)
{
    if(has_received_pose2 || !has_received_pose1) return;
    
    pose2d temp_init_pose(__FLT_MAX__,__FLT_MAX__,__FLT_MAX__);
    tf2::Quaternion q(t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    temp_init_pose.x.x  = t.transform.translation.x;
    temp_init_pose.x.y  = t.transform.translation.y;
    temp_init_pose.theta = yaw;
    initial_poses.push_back(temp_init_pose);
    has_received_pose2 = true;
    RCLCPP_INFO(this->get_logger(), "Got initial pose 2");
    getPaths_and_Publish();

};

/*
----- Function for calculating paths and roadmap and publish -----------
*/

void MissionPlanner::build_roadmap()
{
    // shared_ptr<Map> map (new Map(map_poly));
    point2d t1(-5.0,-5.0);
    point2d t2(-5.0,5.0);
    point2d t3(5.0,5.0);
    point2d t4(5.0,-5.0);
    
    vector<point2d> vec_vert;
    vec_vert.push_back(t1);
    vec_vert.push_back(t2);
    vec_vert.push_back(t3);
    vec_vert.push_back(t4);

    // point2d p1(-1.0, -5.0);
    // point2d p2(-1.0, 2.0);
    // point2d p3(1.0, 2.0);
    // point2d p4(1.0, -5.0);
    
    // vector<point2d> obs_vert;
    // obs_vert.push_back(p1);
    // obs_vert.push_back(p2);
    // obs_vert.push_back(p3);
    // obs_vert.push_back(p4);

    // point2d p21(-2.0, 3.0);
    // point2d p22(-2.0, 5.0);
    // point2d p23(2.0, 5.0);
    // point2d p24(2.0, 3.0);
    
    // vector<point2d> obs_vert2;
    // obs_vert2.push_back(p21);
    // obs_vert2.push_back(p22);
    // obs_vert2.push_back(p23);
    // obs_vert2.push_back(p24);

    

    Polygon test_map(vec_vert); 
    // Polygon test_obs(obs_vert);
    // Polygon test_obs1(obs_vert2);

    shared_ptr<Map> map (new Map(test_map));
    for (int i = -4; i < 4; i+=3)
    {
        for (int j = -5; j < 4; j+=3)
        {
            point2d p1(i,  j);
            point2d p2(i+1, j);
            point2d p3(i+1,j+1);
            point2d p4(i, j+1);
            vector<point2d> check_vert;
            check_vert.push_back(p1);
            check_vert.push_back(p2);
            check_vert.push_back(p3);
            check_vert.push_back(p4);
            Polygon check(check_vert, 0.0); 
            map->addObstacle(check);
        }
    }

    // for (auto i = 0; i < obstacle_list.size(); i++)
    // {
    //     map->addObstacle(obstacle_list[i]);
    // }
    // map->addObstacle(test_obs);
    // map->addObstacle(test_obs1);

    RCLCPP_INFO(this->get_logger(),"Map made and Obstacles included. Free space = %0.2f", map->getFreeSpace());

    d->map = map;
    d->_K = conf->getK();

    std::string planner_type = conf->getPlannerType();

    if (planner_type == "DPRMstar")
    {
        planner = new DPRMstar(map);
    }
    else if (planner_type == "GeometricPRMstar")
    {
        planner = new GeometricPRMstar(map);
    }
    else
    {
        planner = new ExactCell(map);
    }
    planner->config = conf;
    planner->graph->config = conf;
    RCLCPP_INFO(this->get_logger(),"Planner made");
    planner->dCurve = d;
    planner->genRoadmap(conf->getNumPoints(), conf->getNumAngles());
    RCLCPP_INFO(this->get_logger(),"Roadmap Generated");

};

void MissionPlanner::getPaths_and_Publish()
{
    if (path_done) return;
    if(has_received_map && has_received_gate && has_received_obs && has_received_pose1 && has_received_pose2)
    {        
        RCLCPP_INFO(this->get_logger(), "Got all information from simulation");
        RCLCPP_INFO(this->get_logger(), "Calculating Roadmap");
        clock_t beforeTime = clock();
        build_roadmap();
        clock_t afterTime = clock() - beforeTime;
        cout << "Building the roadmap took " <<(float)afterTime/CLOCKS_PER_SEC << " seconds." << endl;        
        for (auto rob = 1; rob <= initial_poses.size(); rob++)
        {
            pose2d gate(2.5, -5, -M_PI);
            cout << "Pose is: " << initial_poses[rob-1].x.x << ", " << initial_poses[rob-1].x.y << ", " << initial_poses[rob-1].theta << endl;
            deque<arcs> path = planner->getPath(initial_poses[rob-1], gates[0]);
            // deque<arcs> path = planner->getPath(initial_poses[rob-1], gate);
            publish_path("shelfino" + to_string(rob) + "/follow_path", path);
        }
        path_done = true;
    }
};

void MissionPlanner::test()
{
    clock_t beforeTime = clock();
    build_roadmap();
    pose2d e(-4.9,-4.9,0.0);
    pose2d e2(4.9,4.9,0.0);
    pose2d e3(4.9,-4.9,0.0);

    deque<arcs> path;
    if (conf->getPlannerType() == "DPRMstar") path = planner->getPath(e,e2);
    else path = planner->getPath(e.x, e2.x);
    cout << " -----------   " << path.size()<< endl;
    auto traj = d->arcs_to_path(path, 0.1);

    clock_t afterTime = clock() - beforeTime;
    float comp_time = (float)afterTime/CLOCKS_PER_SEC;
    float path_length = 0.0;
    for (int i = 0; i < path.size(); i++)
    {
        path_length += path[i].L;
    }
    ofstream myfile;
    myfile.open("path.csv");
    for (int i = 0; i < traj.poses.size(); i++)
    {
        myfile << traj.poses[i].pose.position.x << "," << traj.poses[i].pose.position.y << endl;
    }
    myfile.close();
    myfile.open ("results.txt");
    myfile << comp_time << "," << path_length<< "," << planner->n_connections;
    myfile.close();
    exit(0);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MissionPlanner>());
  rclcpp::shutdown();
  return 0;
};