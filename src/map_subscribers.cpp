#pragma once
#include "geometry.h"
#include "graph.h"
#include "map.h"

#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "obstacles_msgs/msg/obstacle_array_msg.hpp"
#include "obstacles_msgs/msg/obstacle_msg.hpp"
#include "std_msgs/msg/header.hpp"


using std::placeholders::_1;

class ObstacleSubscriber : public rclcpp::Node
{
  private:
    vector<Polygon> obstacle_list;
    void topic_callback(const obstacles_msgs::msg::ObstacleArrayMsg obstacle_message)
    {
      RCLCPP_INFO(this->get_logger(), "further");
      vector<point2d> polygon_input;
      Polygon poly;
      // create polygon obstacle object
      for (int i = 0; i < obstacle_message.obstacles.size(); i++)
      {
        geometry_msgs::msg::Polygon aux = obstacle_message.obstacles[i].polygon;
        int nr_points = aux.points.size();
        for (int j = 0; j < nr_points; j++)
        {
            point2d temp;
            temp.x = float(aux.points[j].x);
            temp.y = float(aux.points[j].y);
            polygon_input.push_back(temp);
            RCLCPP_INFO(this->get_logger(), "Getting obs info");
        }
        poly.verteces = polygon_input;
        obstacle_list.push_back(poly);
      }
    }
  public:
    ObstacleSubscriber()
    : Node("obstacle_subscriber")
    {
      RCLCPP_INFO(this->get_logger(), "Starting obstacle subscriber");
      subscription_ = this->create_subscription<obstacles_msgs::msg::ObstacleArrayMsg>(
      "obstacles", 10, std::bind(&ObstacleSubscriber::topic_callback, this, _1));
    }
    vector<Polygon> getObstacleList() const {return obstacle_list;};

    rclcpp::Subscription<obstacles_msgs::msg::ObstacleArrayMsg>::SharedPtr subscription_;
};

class MapLimitSubscriber : public rclcpp::Node
{
  private:
    Polygon map;
    void topic_callback(const geometry_msgs::msg::Polygon::SharedPtr outline_message)
    {
      RCLCPP_INFO(this->get_logger(), "further map");
      vector<point2d> outer_verteces;
      for (int i = 0; i < outline_message->points.size(); i++)
      {
        point2d temp;
        temp.x = outline_message->points[i].x;
        temp.y = outline_message->points[i].y;
        outer_verteces.push_back(temp);
        RCLCPP_INFO(this->get_logger(), "Getting map info");
      }
      map.verteces = outer_verteces;
    }
    rclcpp::Subscription<geometry_msgs::msg::Polygon>::SharedPtr subscription_;
  public:
    MapLimitSubscriber()
    : Node("maplimit_subscriber")
    {
      RCLCPP_INFO(this->get_logger(), "Starting map subscriber");
      subscription_ = this->create_subscription<geometry_msgs::msg::Polygon>(
      "map_borders", 10, std::bind(&MapLimitSubscriber::topic_callback, this, _1));
    }
    Polygon getMap() const {return map;};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  // rclcpp::spin(std::make_shared<ObstacleSubscriber>());
  rclcpp::spin(std::make_shared<MapLimitSubscriber>());
  rclcpp::shutdown();
  return 0;
}