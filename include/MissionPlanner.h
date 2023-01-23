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

struct combo
{
    vector<arcs> shelfino1Path;
    vector<arcs> shelfino2Path;
    vector<arcs> shelfino3Path;
    float costs[3];
};

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
        void subscribe_to_map_info();
        void subscribe_to_obstacle_info();
        void subscribe_to_gate_info();
        void subscribe_to_shelfino1();
        void subscribe_to_shelfino2();
        void subscribe_to_shelfino3();

        void find_best_paths_combo();
   
    public:
        MissionPlanner()
        : Node("mission_planner"), count_(0)
        {
            print_message();
            conf = std::shared_ptr<ConfigParams>(new ConfigParams("src/dubin/config.txt"));
            d = std::shared_ptr<dubinCurve>(new dubinCurve());
            void subscribe_to_map_info();
            void subscribe_to_obstacle_info();
            void subscribe_to_gate_info();
            void subscribe_to_shelfino1();
            void subscribe_to_shelfino2();
            void subscribe_to_shelfino3();
            // // get initial pose
            // RCLCPP_INFO(this->get_logger(), "Getting position info for shelfino %i", 1);
            // rclcpp::QoS qos_pose = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);

            // pose_subscription_[0] = this->create_subscription<geometry_msgs::msg::TransformStamped>(
            // name, qos_pose, std::bind(&MissionPlanner::pose_topic_callback, this, _1));
        }       
};