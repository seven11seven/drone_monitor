#include "misc/visualizer.hpp"
#include "gcopter/trajectory.hpp"
#include "gcopter/gcopter.hpp"
#include "gcopter/firi.hpp"
#include "gcopter/flatness.hpp"
#include "gcopter/voxel_map.hpp"
#include "gcopter/sfc_gen.hpp"

#include <cmath>
#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include <chrono>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

class MiCModule : public rclcpp::Node
{
public:
  MiCModule()
  : Node("fake_module_node"),
    module_gen(false)
  {
    edgePub = this->create_publisher<visualization_msgs::msg::Marker>("/visualizer/edge", 1000);
    
    moduleGeom = std::vector<Eigen::Vector3d>(8);
    Eigen::Vector3d coGeom(0.0, 0.0, 1.0);
    const double diameter = 1;
    moduleGen(coGeom, diameter, moduleGeom);
    
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000 / 10)),
      std::bind(&MiCModule::timer_callback, this));
  }

  void timer_callback()
  {
    visualizer.visualizeMoudle(moduleGeom, edgePub);
  }

  inline void moduleGen(const Eigen::Vector3d coGeom,
                        const double &diameter,
                        std::vector<Eigen::Vector3d> &module)
  {   
    // Make sure module has enough space
    if (module.size() < 8) module.resize(8);

    Eigen::Vector3d offset = Eigen::Vector3d(diameter, diameter, diameter);

    // cwiseProduct to perform element-wise multiplication between vectors
    module[0] = coGeom + Eigen::Vector3d(1, 1, 1).cwiseProduct(offset);
    module[1] = coGeom + Eigen::Vector3d(-1, 1, 1).cwiseProduct(offset);
    module[2] = coGeom + Eigen::Vector3d(-1, -1, 1).cwiseProduct(offset);
    module[3] = coGeom + Eigen::Vector3d(1, -1, 1).cwiseProduct(offset);
    module[4] = coGeom + Eigen::Vector3d(1, 1, -1).cwiseProduct(offset);
    module[5] = coGeom + Eigen::Vector3d(-1, 1, -1).cwiseProduct(offset);
    module[6] = coGeom + Eigen::Vector3d(-1, -1, -1).cwiseProduct(offset);
    module[7] = coGeom + Eigen::Vector3d(1, -1, -1).cwiseProduct(offset);

    module_gen = true;
  }

public:
  bool module_gen; 
  Visualizer visualizer;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr edgePub;
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<Eigen::Vector3d> moduleGeom;
};

int main(int argc, char **argv)
{   
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MiCModule>());
    rclcpp::shutdown();
    return 0;
}