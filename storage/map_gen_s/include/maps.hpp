#ifndef MAPS_HPP
#define MAPS_HPP

#include <rclcpp/rclcpp.hpp>

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace mocka
{

class Maps : public rclcpp::Node 
{

public:
  typedef struct BasicInfo {
    // ros::NodeHandle *nh_private;
    int sizeX;
    int sizeY;
    int sizeZ;
    int seed;
    double scale;
    sensor_msgs::msg::PointCloud2::SharedPtr output;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
  } BasicInfo;

  BasicInfo getInfo() const;
  void setInfo(const BasicInfo &value);

// Constructor
public:
  Maps(const std::string & node_name): Node(node_name) {};

// Public function
public:
  void generate(int type);

// Private Variable
private:
  BasicInfo info;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_pub;
  size_t count_;

// Private function 
private:
  void pcl2ros();

  void perlin3D();
  void maze2D();
  void randomMapGenerate();
  void Maze3DGen();
  void recursiveDivision(int xl, int xh, int yl, int yh, Eigen::MatrixXi &maze);
  void recursizeDivisionMaze(Eigen::MatrixXi &maze);
  void optimizeMap();

  void timer_callback();
};

class MazePoint 
{

private:
  pcl::PointXYZ point;
  double dist1;
  double dist2;
  int point1;
  int point2;
  bool isdoor;

public:
  pcl::PointXYZ getPoint();
  int getPoint1();
  int getPoint2();
  double getDist1();
  double getDist2();
  void setPoint(pcl::PointXYZ p);
  void setPoint1(int p);
  void setPoint2(int p);
  void setDist1(double set);
  void setDist2(double set);
};

} // namespace mocka

#endif // MAPS_HPP
