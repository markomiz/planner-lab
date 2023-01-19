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
#include "obstacles_msgs/msg/obstacle_array_msg.hpp"
#include "obstacles_msgs/msg/obstacle_msg.hpp"
#include "std_msgs/msg/header.hpp"

#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"

#include "nav2_msgs/action/follow_path.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;

void MissionPlanner::obstacle_topic_callback(const obstacles_msgs::msg::ObstacleArrayMsg obstacle_message)
{
    // create polygon obstacle object
    int n_obs = obstacle_message.obstacles.size();
    RCLCPP_INFO(this->get_logger(), "There are '%i' obstacles", n_obs);
    for (int i = 0; i < n_obs; i++)
    {
        vector<point2d> polygon_input;
        RCLCPP_INFO(this->get_logger(), "Obstacle '%i' points:", i);
        geometry_msgs::msg::Polygon aux = obstacle_message.obstacles[i].polygon;
        int nr_points = aux.points.size();
        for (int j = 0; j < nr_points; j++)
        {
            point2d temp;
            temp.x = float(aux.points[j].x);
            temp.y = float(aux.points[j].y);
            polygon_input.push_back(temp);
            RCLCPP_INFO(this->get_logger(), "Getting obs info: x = '%0.2f', y = '%0.2f'", temp.x, temp.y);
        }
        Polygon poly(polygon_input);
        obstacle_list.push_back(poly);
    }
    is_receive_obs = true;
    if (/*is_receive_map && is_receive_gate &&*/ is_receive_obs){calculations();}
};

void MissionPlanner::map_topic_callback(const geometry_msgs::msg::Polygon outline_message)
{
    vector<point2d> outer_verteces;
    for (int i = 0; i < outline_message.points.size(); i++)
    {
        point2d temp;
        temp.x = outline_message.points[i].x;
        temp.y = outline_message.points[i].y;
        outer_verteces.push_back(temp);        
    }
    map_poly = Polygon(outer_verteces);
    is_receive_map = true;
    if (/*is_receive_map && is_receive_gate &&*/ is_receive_obs){calculations();}
};

void MissionPlanner::gate_topic_callback(const geometry_msgs::msg::Pose outline_message)
{
    tf2::Quaternion q(outline_message.orientation.x, outline_message.orientation.y, outline_message.orientation.z, outline_message.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    gate.x.x  = outline_message.position.x;
    gate.x.y  = outline_message.position.y;
    gate.theta = yaw;
    is_receive_gate = true;
    if (/*is_receive_map && is_receive_gate &&*/ is_receive_obs){calculations();}
};

pose2d MissionPlanner::subscribeToPos(std::string robot_id){
    RCLCPP_INFO(this->get_logger(), "Getting initial pose");
    // RCLCPP_INFO(this->get_logger(), "Frame: %s", robot_id.c_str());
    std::string target_frame_ = this->declare_parameter<std::string>("target_frame", robot_id);
    std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer;

    tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    std::string fromFrameRel = target_frame_.c_str();
    std::string toFrameRel = std::string("map");
    
    geometry_msgs::msg::TransformStamped t;
    pose2d xyth(__FLT_MAX__,__FLT_MAX__,__FLT_MAX__);
    try {
        t = tf_buffer->lookupTransform(toFrameRel, fromFrameRel, tf2::TimePointZero, 5000ms);
    } catch (/*const tf2::TransformException & ex*/...) {
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

void MissionPlanner::do_calculations(pose2d x0)
{
    pose2d xf(5,5,0.5);

    point2d t1(-10,-10);
    point2d t2(-10,10);
    point2d t3(10,10);
    point2d t4(10,-10);

    point2d p1(1,4);
    point2d p2(2,5);
    point2d p3(5,2);
    point2d p4(4,1);
    
    vector<point2d> vec_vert;
    vec_vert.push_back(t1);
    vec_vert.push_back(t2);
    vec_vert.push_back(t3);
    vec_vert.push_back(t4);

    vector<point2d> obs_vert;
    obs_vert.push_back(p1);
    obs_vert.push_back(p2);
    obs_vert.push_back(p3);
    obs_vert.push_back(p4);

    
    Polygon test_map(vec_vert); 
    RCLCPP_INFO(this->get_logger(),"Area %f", test_map.area);
    Polygon test_obstacle(obs_vert);
    cout << "\n  " << test_obstacle.area << " - obstacle area";
    
    shared_ptr<Map> map (new Map(test_map));

    map->addObstacle(test_obstacle);
    
    // for (int i = 0; i < obstacle_list.size(); i++)
    // {
    //     map->addObstacle(obstacle_list[i]);
    // }

    RCLCPP_INFO(this->get_logger(),"Map made and Obstacles included. Free space = %0.2f", map->getFreeSpace());
    

    shared_ptr<dubinCurve> d (new dubinCurve());
    d->map = map;
    d->_K = conf->getK();

    
    planner = new PRMstar(map);
    RCLCPP_INFO(this->get_logger(),"Planner made");
    planner->dCurve = d;
    planner->genRoadmapPlus(conf->getNumPoints(), conf->getNumAngles());
    RCLCPP_INFO(this->get_logger(),"Roadmap Generated");

    deque<arcs> way = planner->getPath(x0,xf);
    path = d->arcs_to_path(way, 0.05);
    RCLCPP_INFO(this->get_logger(),"Path found");

    // nav_msgs::msg::Path path =  d->generatePathFromDubins(x0, d->calculateMultiPoint(x0, x1, mids, 12), delta);
    // path.header.stamp = this->get_clock()->now();
};

void MissionPlanner::testSinglePath(pose2d x0)
{
    pose2d xf(-0.1,0,-0.1);

    point2d t1(-10,-10);
    point2d t2(-10,10);
    point2d t3(10,10);
    point2d t4(10,-10);

    point2d p1(0.1,-5);
    point2d p2(0.1,5);
    point2d p3(0.2,5);
    point2d p4(0.2,-5);
    
    vector<point2d> vec_vert;
    vec_vert.push_back(t1);
    vec_vert.push_back(t2);
    vec_vert.push_back(t3);
    vec_vert.push_back(t4);

    vector<point2d> obs_vert;
    obs_vert.push_back(p1);
    obs_vert.push_back(p2);
    obs_vert.push_back(p3);
    obs_vert.push_back(p4);

    
    Polygon test_map(vec_vert); 
    RCLCPP_INFO(this->get_logger(),"Area %f", test_map.area);
    Polygon test_obstacle(obs_vert);
    cout << "\n  " << test_obstacle.area << " - obstacle area";
    
    shared_ptr<Map> map (new Map(test_map));

    map->addObstacle(test_obstacle);
    RCLCPP_INFO(this->get_logger(),"Map made and Obstacles included. Free space = %0.2f", map->getFreeSpace());

    shared_ptr<dubinCurve> d (new dubinCurve());
    d->map = map;
    d->_K = conf->getK();

    // // test single path
    dubins_params dp = d->calculateSinglePath(x0, xf);
    arcs Ao = arcs(x0, dp);
    if (map->colliding(Ao)){
        cout << "\n the collision seems ligit";
    }
    arcs A = Ao.get_inverse();
    if (map->colliding(A)){
        cout << "\n the collision seems ligit for inverse too";
    }
    if (map->colliding(Ao)){
        cout << "\n the collision seems ligit";
    }
    else cout << "\n not colliding anymore?";
    cout << "\nwhat..";
    ofstream myfile ("path_points.txt");
    pose2d currentPoint = A.a[0].start;
    myfile << currentPoint.x.x << "; " << currentPoint.x.y << "\n";
    int co = 0;
    for (auto j = 0; j < 3; j++)
    {
        arc &a = A.a[j];

        for (float ds = 0; ds < a.s; ds += 0.01)
        {
          currentPoint = arc::next_pose(a.start, ds, a.K);
          myfile << currentPoint.x.x << "; " << currentPoint.x.y << "\n"; 
          co++;
        }
        currentPoint = arc::next_pose(a.start, a.s, a.K);
        myfile << currentPoint.x.x << "; " << currentPoint.x.y << "\n";
    }
    return;
}

void MissionPlanner::publish_results()
{
    /*
        ACTION CLIENT AND SERVER PART
    */

    publisher_ = this->create_publisher<nav_msgs::msg::Path>("/plan", 10);

    using FollowPath = nav2_msgs::action::FollowPath;

    rclcpp_action::Client<FollowPath>::SharedPtr client_ptr_;

    client_ptr_ = rclcpp_action::create_client<FollowPath>(this,"follow_path");

    if (!client_ptr_->wait_for_action_server()) {
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
    rclcpp::shutdown();
};

void MissionPlanner::calculations()
{
    RCLCPP_INFO(this->get_logger(), "Im in the if");
    while(true){
        // Define robot currently working
        name = "shelfino" + std::to_string(robot_numb) + "/base_link"; 
        RCLCPP_INFO(this->get_logger(), "Working on shelfino %i", robot_numb);
        RCLCPP_INFO(this->get_logger(), "%s", name.c_str());
        // get initial pose
        // pose2d temp = subscribeToPos(name);
        pose2d temp(0,0,0);
        if (temp.x.x == __FLT_MAX__ && temp.x.y == __FLT_MAX__ && temp.theta == __FLT_MAX__)
        {
            break;
        }
        robot_numb++;
        RCLCPP_INFO(this->get_logger(), "Calculating");
        do_calculations(temp);

        // Publish results
        RCLCPP_INFO(this->get_logger(), "Publishing path");
        publish_results();
        //do some computations and publish messages
    }
    rclcpp::shutdown();
};



int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MissionPlanner>());
  rclcpp::shutdown();
  return 0;
};