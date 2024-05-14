#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <algorithm>
#include <iostream>
#include <vector>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include "maps.hpp"


int
main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto maps_node = std::make_shared<mocka::Maps>("mockamap");
    
    ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("mock_map", 1);
    pcl::PointCloud<pcl::PointXYZ> cloud;
    sensor_msgs::msg::PointCloud2  output;
    // Fill in the cloud data

    int seed;

    int sizeX;
    int sizeY;
    int sizeZ;

    double scale;
    double update_freq;

    int type;

    nh_private.param("seed", seed, 4546);
    nh_private.param("update_freq", update_freq, 1.0);
    nh_private.param("resolution", scale, 0.38);
    nh_private.param("x_length", sizeX, 100);
    nh_private.param("y_length", sizeY, 100);
    nh_private.param("z_length", sizeZ, 10);

    nh_private.param("type", type, 1);

    scale = 1 / scale;
    sizeX = sizeX * scale;
    sizeY = sizeY * scale;
    sizeZ = sizeZ * scale;

    mocka::Maps::BasicInfo info;
    info.nh_private = &nh_private;
    info.sizeX      = sizeX;
    info.sizeY      = sizeY;
    info.sizeZ      = sizeZ;
    info.seed       = seed;
    info.scale      = scale;
    info.output     = &output;
    info.cloud      = &cloud;

    mocka::Maps map;
    map.setInfo(info);
    map.generate(type);

    //  optimizeMap(info);

    //! @note publish loop
    ros::Rate loop_rate(update_freq);
    while (ros::ok())
    {
      pcl_pub.publish(output);
      ros::spinOnce();
      loop_rate.sleep();
    }
    return 0;
}
