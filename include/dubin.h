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
#include "config_server.h"

using namespace std::chrono_literals;

class Dubin : public rclcpp::Node
{

public:
  Dubin()
: Node("dubin_publisher"), count_(0)
{
  publisher_ = this->create_publisher<nav_msgs::msg::Path>("/plan", 10);
  // timer_ = this->create_wall_timer(
  //   500ms, std::bind(&Dubin::timer_callback, this));
  conf = std::shared_ptr<ConfigParams>(new ConfigParams("src/dubin/config.txt"));
  this->timer_callback();

};
  ~Dubin(){};

  pose2d subscribeToPos();

private:
  void timer_callback();
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_;
  std::shared_ptr<ConfigParams> conf;
  size_t count_;
  PRMstar* planner;
  void print_message()
    {
      std::cout << "\n__________                    .__                   __  .__                          \n\\______   \\__ __  ____   ____ |__| ____    ____   _/  |_|  |__   ____                \n |       _/  |  \\/    \\ /    \\|  |/    \\  / ___\\  \\   __\\  |  \\_/ __ \\               \n |    |   \\  |  /   |  \\   |  \\  |   |  \\/ /_/  >  |  | |   Y  \\  ___/               \n |____|_  /____/|___|  /___|  /__|___|  /\\___  /   |__| |___|  /\\___  >              \n        \\/           \\/     \\/        \\//_____/              \\/     \\/               \n__________                 __    __________.__                                    ._.\n\\______   \\ ____   _______/  |_  \\______   \\  | _____    ____   ____   ___________| |\n |    |  _// __ \\ /  ___/\\   __\\  |     ___/  | \\__  \\  /    \\ /    \\_/ __ \\_  __ \\ |\n |    |   \\  ___/ \\___ \\  |  |    |    |   |  |__/ __ \\|   |  \\   |  \\  ___/|  | \\/\\|\n |______  /\\___  >____  > |__|    |____|   |____(____  /___|  /___|  /\\___  >__|   __\n        \\/     \\/     \\/                             \\/     \\/     \\/     \\/       \\/\n";
    };

  
};


