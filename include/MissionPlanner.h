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
#include "geometry_msgs/msg/polygon_stamped.hpp"

#include "geometry.h"
#include "PRMstar.h"
#include "graph.h"
#include "PRMstar.h"
#include "map.h"
#include "config_server.h"

#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "obstacles_msgs/msg/obstacle_array_msg.hpp"
#include "obstacles_msgs/msg/obstacle_msg.hpp"
#include "std_msgs/msg/header.hpp"

using std::placeholders::_1;

class MissionPlanner : public rclcpp::Node
{
    private:
        // Input variables

        vector<Polygon> obstacle_list;
        Polygon map_poly;
        vector<pose2d> gate;
        PRMstar* planner;
        nav_msgs::msg::Path path;
        vector<pose2d> initial_pose;
        shared_ptr<dubinCurve> d;
        

        rclcpp::Subscription<obstacles_msgs::msg::ObstacleArrayMsg>::SharedPtr obs_subscription_;
        rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr map_subscription_;
        rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr gate_subscription_;
        rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr pose_subscription_[3];
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_;

        rclcpp::TimerBase::SharedPtr timer_;
        std::shared_ptr<ConfigParams> conf;
        size_t count_;
        std::string name;
        int is_receive_map = true;
        int is_receive_gate = false;
        int is_receive_obs = false;
        int poses_received = 0;
        
        
        //Subscribers
        void obstacle_topic_callback(const obstacles_msgs::msg::ObstacleArrayMsg obstacle_message);

        void map_topic_callback(const geometry_msgs::msg::PolygonStamped outline_message);
        
        void gate_topic_callback(const geometry_msgs::msg::PoseArray outline_message);

        void pose_topic_callback(const geometry_msgs::msg::TransformStamped outline_message);

        void build_roadmap();

        void publish_results(int robot);
        
        void print_message()
        {
            std::cout << "\n__________                    .__                   __  .__                          \n\\______   \\__ __  ____   ____ |__| ____    ____   _/  |_|  |__   ____                \n |       _/  |  \\/    \\ /    \\|  |/    \\  / ___\\  \\   __\\  |  \\_/ __ \\               \n |    |   \\  |  /   |  \\   |  \\  |   |  \\/ /_/  >  |  | |   Y  \\  ___/               \n |____|_  /____/|___|  /___|  /__|___|  /\\___  /   |__| |___|  /\\___  >              \n        \\/           \\/     \\/        \\//_____/              \\/     \\/               \n__________                 __    __________.__                                    ._.\n\\______   \\ ____   _______/  |_  \\______   \\  | _____    ____   ____   ___________| |\n |    |  _// __ \\ /  ___/\\   __\\  |     ___/  | \\__  \\  /    \\ /    \\_/ __ \\_  __ \\ |\n |    |   \\  ___/ \\___ \\  |  |    |    |   |  |__/ __ \\|   |  \\   |  \\  ___/|  | \\/\\|\n |______  /\\___  >____  > |__|    |____|   |____(____  /___|  /___|  /\\___  >__|   __\n        \\/     \\/     \\/                             \\/     \\/     \\/     \\/       \\/\n";
        };


        void getPaths_and_Publish();
   
    public:
        MissionPlanner()
        : Node("mission_planner"), count_(0)
        {
            
            print_message();

            conf = std::shared_ptr<ConfigParams>(new ConfigParams("src/dubin/config.txt"));
            d = std::shared_ptr<dubinCurve>(new dubinCurve());
            build_roadmap();
            return;

            // // Create subscribers for gate, map and obstacles
            // RCLCPP_INFO(this->get_logger(), "Getting obstacle info");
            // rclcpp::QoS qos_obs = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);
            // obs_subscription_ = this->create_subscription<obstacles_msgs::msg::ObstacleArrayMsg>(
            // "obstacles", qos_obs, std::bind(&MissionPlanner::obstacle_topic_callback, this, _1));

            // RCLCPP_INFO(this->get_logger(), "Getting map info");
            // rclcpp::QoS qos_map = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);
            // map_subscription_ = this->create_subscription<geometry_msgs::msg::PolygonStamped>(
            // "map_borders", qos_map, std::bind(&MissionPlanner::map_topic_callback, this, _1));

            // RCLCPP_INFO(this->get_logger(), "Getting gate info");
            // rclcpp::QoS qos_gate = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);
            // gate_subscription_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            // "gate_position", qos_gate, std::bind(&MissionPlanner::gate_topic_callback, this, _1));
            
            // for (int i = 0; i < 2; i++)
            // {
            //     // Define robot currently working
            //     name = "shelfino" + std::to_string(i + 1) + "/transform"; 
            //     // RCLCPP_INFO(this->get_logger(), "%s", name.c_str());
                
            //     // get initial pose
            //     RCLCPP_INFO(this->get_logger(), "Getting position info for shelfino %i", i);
            //     rclcpp::QoS qos_pose = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);

            //     pose_subscription_[i] = this->create_subscription<geometry_msgs::msg::TransformStamped>(
            //     name, qos_pose, std::bind(&MissionPlanner::pose_topic_callback, this, _1));
            // }
            // Define robot currently working
            // name = "shelfino" + std::to_string(robot_numb+1) + "/transform"; 
            // // RCLCPP_INFO(this->get_logger(), "%s", name.c_str());
            
            // // get initial pose
            // RCLCPP_INFO(this->get_logger(), "Getting position info for shelfino %i", 1);
            // rclcpp::QoS qos_pose = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);

            // pose_subscription_[0] = this->create_subscription<geometry_msgs::msg::TransformStamped>(
            // name, qos_pose, std::bind(&MissionPlanner::pose_topic_callback, this, _1));
        }       
};