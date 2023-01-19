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
#include "PRMstar.h"
#include "graph.h"
#include "PRMstar.h"
#include "map.h"
#include "config_server.h"

#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/polygon.hpp"
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
        pose2d gate;
        PRMstar* planner;
        nav_msgs::msg::Path path;
        

        rclcpp::Subscription<obstacles_msgs::msg::ObstacleArrayMsg>::SharedPtr obs_subscription_;
        rclcpp::Subscription<geometry_msgs::msg::Polygon>::SharedPtr map_subscription_;
        rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr gate_subscription_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_;

        rclcpp::TimerBase::SharedPtr timer_;
        std::shared_ptr<ConfigParams> conf;
        size_t count_;
        int robot_numb = 1;
        std::string name;
        int is_receive_map = true;
        int is_receive_gate = true;
        int is_receive_obs = false;
        
        
        //Subscribers
        void obstacle_topic_callback(const obstacles_msgs::msg::ObstacleArrayMsg obstacle_message);

        void map_topic_callback(const geometry_msgs::msg::Polygon outline_message);
        
        void gate_topic_callback(const geometry_msgs::msg::Pose outline_message);
        
        pose2d subscribeToPos(std::string robot_id);

        void do_calculations(pose2d x0);

        void publish_results();
        
        void print_message()
        {
            std::cout << "\n__________                    .__                   __  .__                          \n\\______   \\__ __  ____   ____ |__| ____    ____   _/  |_|  |__   ____                \n |       _/  |  \\/    \\ /    \\|  |/    \\  / ___\\  \\   __\\  |  \\_/ __ \\               \n |    |   \\  |  /   |  \\   |  \\  |   |  \\/ /_/  >  |  | |   Y  \\  ___/               \n |____|_  /____/|___|  /___|  /__|___|  /\\___  /   |__| |___|  /\\___  >              \n        \\/           \\/     \\/        \\//_____/              \\/     \\/               \n__________                 __    __________.__                                    ._.\n\\______   \\ ____   _______/  |_  \\______   \\  | _____    ____   ____   ___________| |\n |    |  _// __ \\ /  ___/\\   __\\  |     ___/  | \\__  \\  /    \\ /    \\_/ __ \\_  __ \\ |\n |    |   \\  ___/ \\___ \\  |  |    |    |   |  |__/ __ \\|   |  \\   |  \\  ___/|  | \\/\\|\n |______  /\\___  >____  > |__|    |____|   |____(____  /___|  /___|  /\\___  >__|   __\n        \\/     \\/     \\/                             \\/     \\/     \\/     \\/       \\/\n";
        };

        void calculations();

        void testSinglePath(pose2d x0);
    
    public:
        MissionPlanner()
        : Node("mission_planner"), count_(0)
        {
            print_message();

            conf = std::shared_ptr<ConfigParams>(new ConfigParams("src/dubin/config.txt"));
            
            testSinglePath(pose2d(0,0,0));
            return;

            // // Create subscribers for gate, map and obstacles
            // RCLCPP_INFO(this->get_logger(), "Getting obstacle info");
            // obs_subscription_ = this->create_subscription<obstacles_msgs::msg::ObstacleArrayMsg>(
            // "obstacles", 10, std::bind(&MissionPlanner::obstacle_topic_callback, this, _1));

            // RCLCPP_INFO(this->get_logger(), "Getting map info");
            // map_subscription_ = this->create_subscription<geometry_msgs::msg::Polygon>(
            // "map_borders", 10, std::bind(&MissionPlanner::map_topic_callback, this, _1));

            // RCLCPP_INFO(this->get_logger(), "Getting gate info");
            // gate_subscription_ = this->create_subscription<geometry_msgs::msg::Pose>(
            // "gate_position", 10, std::bind(&MissionPlanner::gate_topic_callback, this, _1));
        }       
};