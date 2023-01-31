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
#include "graph.h"
#include "Planner.h"
#include "map.h"
#include "config_server.h"

#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "obstacles_msgs/msg/obstacle_array_msg.hpp"
#include "obstacles_msgs/msg/obstacle_msg.hpp"
#include "std_msgs/msg/header.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using std::placeholders::_1;
using FollowPath = nav2_msgs::action::FollowPath;

class MissionPlanner : public rclcpp::Node
{
    private:
        // Input variables

        vector<Polygon> obstacle_list;
        Polygon map_poly;
        vector<pose2d> gates;
        Planner* planner;

        bool path_done;
        vector<pose2d> initial_poses;
        shared_ptr<dubinCurve> d;

        rclcpp::Subscription<obstacles_msgs::msg::ObstacleArrayMsg>::SharedPtr obs_subscription_;
        rclcpp::Subscription<geometry_msgs::msg::Polygon>::SharedPtr map_subscription_;
        rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr gate_subscription_;
        rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr pose1_subscription_;
        rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr pose2_subscription_;

        rclcpp::TimerBase::SharedPtr timer_;
        std::shared_ptr<ConfigParams> conf;
        size_t count_;

        bool has_received_map = false;
        bool has_received_gate = false;
        bool has_received_obs = false;
        bool has_received_pose1 = false;
        bool has_received_pose2 = false;

        
        //Subscribers
        void obstacle_topic_callback(const obstacles_msgs::msg::ObstacleArrayMsg obstacle_message);
        void map_topic_callback(const geometry_msgs::msg::Polygon outline_message);
        void gate_topic_callback(const geometry_msgs::msg::PoseArray outline_message);
        void pose1_topic_callback(const geometry_msgs::msg::TransformStamped outline_message);
        void pose2_topic_callback(const geometry_msgs::msg::TransformStamped outline_message);

        void build_roadmap();

        // void publish_results(int robot);
        
        void subscribe_to_map();
        void subscribe_to_obstacles();
        void subscribe_to_gate();
        void subscribe_to_pose1();
        void subscribe_to_pose2();

        void print_message()
        {
            std::cout << "\n__________                    .__                   __  .__                          \n\\______   \\__ __  ____   ____ |__| ____    ____   _/  |_|  |__   ____                \n |       _/  |  \\/    \\ /    \\|  |/    \\  / ___\\  \\   __\\  |  \\_/ __ \\               \n |    |   \\  |  /   |  \\   |  \\  |   |  \\/ /_/  >  |  | |   Y  \\  ___/               \n |____|_  /____/|___|  /___|  /__|___|  /\\___  /   |__| |___|  /\\___  >              \n        \\/           \\/     \\/        \\//_____/              \\/     \\/               \n__________                 __    __________.__                                    ._.\n\\______   \\ ____   _______/  |_  \\______   \\  | _____    ____   ____   ___________| |\n |    |  _// __ \\ /  ___/\\   __\\  |     ___/  | \\__  \\  /    \\ /    \\_/ __ \\_  __ \\ |\n |    |   \\  ___/ \\___ \\  |  |    |    |   |  |__/ __ \\|   |  \\   |  \\  ___/|  | \\/\\|\n |______  /\\___  >____  > |__|    |____|   |____(____  /___|  /___|  /\\___  >__|   __\n        \\/     \\/     \\/                             \\/     \\/     \\/     \\/       \\/\n";
        };
        void getPaths_and_Publish();
        void publish_path(string topic, deque<arcs> path);
        void test();
   
    public:
        MissionPlanner()
        : Node("mission_planner"), count_(0)
        {
            print_message();
            path_done = false;
            conf = std::shared_ptr<ConfigParams>(new ConfigParams("config.txt"));
            d = std::shared_ptr<dubinCurve>(new dubinCurve());
            // subscribe_to_map();
            // subscribe_to_obstacles();
            // subscribe_to_gate();
            // subscribe_to_pose1();
            // subscribe_to_pose2();
            test();
        }       
};