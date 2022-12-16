#pragma once
#include "geometry.h"
#include "graph.h"
#include "map.h"
#include "dubinCurve.h"

#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/msg/costmap.hpp"


using std::placeholders::_1;

class ObstacleSubscriber : public rclcpp::Node
{
  public:
    ObstacleSubscriber()
    : Node("obstacle_subscriber")
    {
      subscription_ = this->create_subscription<costmap_converter::msg::ObstacleArrayMsg>(
      "obstacles", 10, std::bind(&ObstacleSubscriber::topic_callback, this, _1));
    }
    vector<Polygon> getObstacleList() {return obstacle_list;};

  private:
    vector<Polygon> obstacle_list;

    void topic_callback(const costmap_converter::msg::ObstacleArrayMsg::SharedPtr obstacle_message) const
    {
        // create polygon obstacle object
        for (int i = 0; i < obstacle_message->obstacles.size(); i++)
        {
            Polygon poly;
            vector<point2d> polygon_input;

            geometry_msgs::msg::Polygon::SharedPtr aux = obstacle_message->obstacles[i].polygon;
            int nr_points = aux->points.size();

            if (nr_points == 1)
            {
                poly.radius = obstacle_message->obstacles.radius;
            }
            for (int j = 0; j < nr_points; j++)
            {
                point2d temp;
                temp.x = float(aux->points[j].x);
                temp.y = float(aux->points[j].y);
                polygon_input.push_back(temp);
            }
            poly.verteces = polygon_input;
            obstacle_list.push_back(poly);
        }
    }

    rclcpp::Subscription<costmap_converter::msg::ObstacleArrayMsg>::SharedPtr subscription_;
};




class MapLimitSubscriber : public rclcpp::Node
{
  public:
    MapLimitSubscriber()
    : Node("maplimit_subscriber")
    {
      subscription_ = this->create_subscription<geometry_msgs::msg::Polygon>(
      "obstacles", 10, std::bind(&MapLimitSubscriber::topic_callback, this, _1));
    }
    Polygon getMap() {return map;};

  private:
    Polygon map;
    void topic_callback(const geometry_msgs::msg::Polygon::SharedPtr outline_message) const
    {
      vector<point2d> outer_verteces;
      for (int i = 0; i < outline_message->points.size(); i++)
      {
          point2d temp;
          temp.x = outline_message->points[i].x;
          temp.y = outline_message->points[i].y;
          outer_verteces.push_back(temp);
      }
      map.verteces = outer_verteces;
    }
    rclcpp::Subscription<costmap_converter::msg::ObstacleArrayMsg>::SharedPtr subscription_;
};