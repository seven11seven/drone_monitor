#ifndef MAZEPOINT_HPP
#define MAZEPOINT_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

class MazePoint 
{
public:
  pcl::PointXYZ getPoint()
  {
    return point;
  }

  int getPoint1()
  {
    return point1;
  }

  int getPoint2()
  {
    return point2;
  }

  double getDist1()
  {
    return dist1;
  }

  double getDist2()
  {
    return dist2;
  }

  void setPoint(pcl::PointXYZ p)
  {
    point = p;
  }

  void setPoint1(int p)
  {
    point1 = p;
  }

  void setPoint2(int p)
  {
    point2 = p;
  }

  void setDist1(double set)
  {
    dist1 = set;
  }

  void setDist2(double set)
  {
    dist2 = set;
  }

private:
  pcl::PointXYZ point;
  double dist1;
  double dist2;
  int point1;
  int point2;
  bool isdoor;

};

#endif