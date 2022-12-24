#pragma once
#include "geometry.h"
#include "graph.h"
#include "map.h"
#include "dubinCurve.h"

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
    vector<point2d> polygon_input;
    Polygon poly = polygon_input;
    void topic_callback(const obstacles_msgs::msg::ObstacleArrayMsg obstacle_message)
    {
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
          }
          poly.verteces = polygon_input;
          obstacle_list.push_back(poly);
        }
    }
  public:
    ObstacleSubscriber()
    : Node("obstacle_subscriber")
    {
      subscription_ = this->create_subscription<obstacles_msgs::msg::ObstacleArrayMsg>(
      "obstacles", 10, std::bind(&ObstacleSubscriber::topic_callback, this, _1));
    }
    vector<Polygon> getObstacleList() const {return obstacle_list;};

    rclcpp::Subscription<obstacles_msgs::msg::ObstacleArrayMsg>::SharedPtr subscription_;
};

class MapLimitSubscriber : public rclcpp::Node
{
  private:
    vector<point2d> outer_verteces;
    Polygon map = outer_verteces;
    void topic_callback(const obstacles_msgs::msg::ObstacleMsg outline_message)
    {
      for (int i = 0; i < outline_message.polygon.points.size(); i++)
      {
        point2d temp;
        temp.x = outline_message.polygon.points[i].x;
        temp.y = outline_message.polygon.points[i].y;
        outer_verteces.push_back(temp);
      }
      map.verteces = outer_verteces;
    }
    rclcpp::Subscription<obstacles_msgs::msg::ObstacleMsg>::SharedPtr subscription_;
  public:
    MapLimitSubscriber()
    : Node("maplimit_subscriber")
    {
      subscription_ = this->create_subscription<obstacles_msgs::msg::ObstacleMsg>(
      "obstacles", 10, std::bind(&MapLimitSubscriber::topic_callback, this, _1));
    }
    Polygon getMap() const {return map;};

};