#include <algorithm>
#include <iostream>
#include <random>
#include <vector>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <Eigen/Core>

#include "perlinnoise.hpp"
#include "mazepoint.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace std::chrono_literals;

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

public:
  Maps() 
  : Node("mockamap_node"), count_(0)
  {
    // Declare paramters before use them
    this->declare_parameter<double>("width_min");
    this->declare_parameter<double>("width_max");
    this->declare_parameter<int>("obstacle_number");
    this->declare_parameter<double>("complexity");
    this->declare_parameter<double>("fill");
    this->declare_parameter<int>("fractal");
    this->declare_parameter<double>("attenuation");
    this->declare_parameter<double>("road_width");
    this->declare_parameter<int>("add_wall_x");
    this->declare_parameter<int>("add_wall_y");
    this->declare_parameter<int>("maze_type");
    this->declare_parameter<int>("numNodes");
    this->declare_parameter<double>("connectivity");
    this->declare_parameter<int>("nodeRad");
    this->declare_parameter<int>("roadRad");

    this->declare_parameter<int>("seed");
    this->declare_parameter<double>("update_freq");
    this->declare_parameter<double>("resolution");
    this->declare_parameter<int>("x_length");
    this->declare_parameter<int>("y_length");
    this->declare_parameter<int>("z_length");
    this->declare_parameter<int>("type");

    // Fill in the cloud data
    int seed;
    int sizeX;
    int sizeY;
    int sizeZ;
    double scale;
    double update_freq;
    int type;

    this->get_parameter_or("seed", seed, 4546);
    this->get_parameter_or("update_freq", update_freq, 1.0);
    this->get_parameter_or("resolution", scale, 0.38);
    this->get_parameter_or("x_length", sizeX, 100);
    this->get_parameter_or("y_length", sizeY, 100);
    this->get_parameter_or("z_length", sizeZ, 10);
    this->get_parameter_or("type", type, 1);

    scale = 1 / scale;
    sizeX = sizeX * scale;
    sizeY = sizeY * scale;
    sizeZ = sizeZ * scale;

    // Initialize the BasicInfo
    info.sizeX      = sizeX;
    info.sizeY      = sizeY;
    info.sizeZ      = sizeZ;
    info.seed       = seed;
    info.scale      = scale;
    info.output     = sensor_msgs::msg::PointCloud2::SharedPtr(new sensor_msgs::msg::PointCloud2);
    info.cloud      = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

    // generate map
    this->generate(type);

    // this->optimizeMap();

    // Set periodically publisher
    // publish loop
    pcl_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("mock_map", 1);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000 / update_freq)),
      std::bind(&Maps::timer_callback, this));
  }

private:
  void timer_callback()
  {
    pcl_pub->publish(*info.output);
  }

public:

  void generate(int type)
  {
    switch (type)
    {
      default:
      case 1:
        perlin3D();
        break;
      case 2:
        randomMapGenerate();
        break;
      case 3:
        std::srand(info.seed);
        maze2D();
        break;
      case 4: // generating 3d maze
        std::srand(info.seed);
        Maze3DGen();
        break;
      case 5:
        RandomMapGenerateCylinder();
        break;
    }
  }

  void pcl2ros()
  {
    pcl::toROSMsg(*info.cloud, *info.output);
    info.output->header.frame_id = "odom";
    RCLCPP_INFO(this->get_logger(), 
                "finish: infill %lf%%",
                info.cloud->width / (1.0 * info.sizeX * info.sizeY * info.sizeZ));
  }

  void perlin3D()
  {
    // The mapping from parameters to generated points cloud
    double complexity;
    double fill;
    int    fractal;
    double attenuation;

    this->get_parameter_or("complexity", complexity, 0.142857);
    this->get_parameter_or("fill", fill, 0.38);
    this->get_parameter_or("fractal", fractal, 1);
    this->get_parameter_or("attenuation", attenuation, 0.5);
    
    // width and height are two important parameters to define the shape of a point cloud in PCL
    // be set based on how the point cloud data is organized (data structure not data)
    info.cloud->width  = info.sizeX * info.sizeY * info.sizeZ;
    info.cloud->height = 1;
    // pre-allocate memory for a certain number of points
    info.cloud->points.resize(info.cloud->width * info.cloud->height);

    PerlinNoise noise(info.seed);

    // allocates a new `std::vector` of `double` on the heap and stores a pointer to it in `v`.
    std::vector<double>* v = new std::vector<double>;
    // pre-allocates memory in the `std::vector` pointed to by `v` 
    // to accommodate `info.cloud->width` number of elements without triggering reallocation.
    // By pre-allocating memory, you can avoid unnecessary reallocations and improve performance.
    v->reserve(info.cloud->width);
    for (int i = 0; i < info.sizeX; ++i)
    {
      for (int j = 0; j < info.sizeY; ++j)
      {
        for (int k = 0; k < info.sizeZ; ++k)
        {
          double tnoise = 0;
          for (int it = 1; it <= fractal; ++it)
          {
            int    dfv = pow(2, it);
            double ta  = attenuation / it;
            tnoise += ta * noise.noise(dfv * i * complexity,
                                       dfv * j * complexity,
                                       dfv * k * complexity);
          }
          v->push_back(tnoise);
        }
      }
    }
    // sorting the elements in the `std::vector` pointed to by the pointer `v`.
    // in ascending order.
    std::sort(v->begin(), v->end());
    int    tpos = info.cloud->width * (1 - fill);
    // The `at` function is a safe way to access elements in a vector with bounds checking. 
    double tmp  = v->at(tpos);
    RCLCPP_INFO(this->get_logger(), "threshold: %lf", tmp);

    int pos = 0;
    for (int i = 0; i < info.sizeX; ++i)
    {
      for (int j = 0; j < info.sizeY; ++j)
      {
        for (int k = 0; k < info.sizeZ; ++k)
        {
          double tnoise = 0;
          for (int it = 1; it <= fractal; ++it)
          {
            int    dfv = pow(2, it);
            double ta  = attenuation / it;
            tnoise += ta * noise.noise(dfv * i * complexity,
                                      dfv * j * complexity,
                                      dfv * k * complexity);
          }
          if (tnoise > tmp)
          {
            info.cloud->points[pos].x =
              i / info.scale - info.sizeX / (2 * info.scale);
            info.cloud->points[pos].y =
              j / info.scale - info.sizeY / (2 * info.scale);
            info.cloud->points[pos].z = k / info.scale;
            pos++;
          }
        }
      }
    }
    info.cloud->width = pos;
    RCLCPP_INFO(this->get_logger(), "the number of points before optimization is %d", info.cloud->width);
    info.cloud->points.resize(info.cloud->width * info.cloud->height);
    pcl2ros();
  }

  void randomMapGenerate()
  {

    std::default_random_engine eng(info.seed);

    double _resolution = 1 / info.scale;

    double _x_l = -info.sizeX / (2 * info.scale);
    double _x_h = info.sizeX / (2 * info.scale);
    double _y_l = -info.sizeY / (2 * info.scale);
    double _y_h = info.sizeY / (2 * info.scale);
    double _h_l = 0;
    double _h_h = info.sizeZ / info.scale;

    double _w_l, _w_h;
    int    _ObsNum;

    this->get_parameter_or("width_min", _w_l, 0.6);
    this->get_parameter_or("width_max", _w_h, 1.5);
    this->get_parameter_or("obstacle_number", _ObsNum, 10);

    std::uniform_real_distribution<double> rand_x;
    std::uniform_real_distribution<double> rand_y;
    std::uniform_real_distribution<double> rand_w;
    std::uniform_real_distribution<double> rand_h;

    pcl::PointXYZ pt_random;

    rand_x = std::uniform_real_distribution<double>(_x_l, _x_h);
    rand_y = std::uniform_real_distribution<double>(_y_l, _y_h);
    rand_w = std::uniform_real_distribution<double>(_w_l, _w_h);
    rand_h = std::uniform_real_distribution<double>(_h_l, _h_h);

    for (int i = 0; i < _ObsNum; i++)
    {
      double x, y;
      x = rand_x(eng);
      y = rand_y(eng);

      double w, h;
      w = rand_w(eng);
      h = rand_h(eng);

      int widNum = ceil(w / _resolution);
      int heiNum = ceil(h / _resolution);

      int rl, rh, sl, sh;
      rl = -widNum / 2;
      rh = widNum / 2;
      sl = -widNum / 2;
      sh = widNum / 2;

      for (int r = rl; r < rh; r++)
        for (int s = sl; s < sh; s++)
        {
          for (int t = 0; t < heiNum; t++)
          {
            if ((r - rl) * (r - rh + 1) * (s - sl) * (s - sh + 1) * t *
                  (t - heiNum + 1) ==
                0)
            {
              pt_random.x = x + r * _resolution;
              pt_random.y = y + s * _resolution;
              pt_random.z = t * _resolution;
              info.cloud->points.push_back(pt_random);
            }
          }
        }
    }

    info.cloud->width    = info.cloud->points.size();
    info.cloud->height   = 1;
    info.cloud->is_dense = true;

    pcl2ros();
  }

  void maze2D()
  {
    double width;
    int    type;
    int    addWallX;
    int    addWallY;
    this->get_parameter_or("road_width", width, 1.0);
    this->get_parameter_or("add_wall_x", addWallX, 0);
    this->get_parameter_or("add_wall_y", addWallY, 0);
    this->get_parameter_or("maze_type", type, 1);

    int mx = info.sizeX / (width * info.scale);
    int my = info.sizeY / (width * info.scale);

    Eigen::MatrixXi maze(mx, my);
    maze.setZero();

    switch (type)
    {
      case 1:
        recursiveDivision(0, maze.cols() - 1, 0, maze.rows() - 1, maze);
        break;
    }

    if (addWallX)
    {
      for (int i = 0; i < mx; ++i)
      {
        maze(i, 0)      = 1;
        maze(i, my - 1) = 1;
      }
    }
    if (addWallY)
    {
      for (int i = 0; i < my; ++i)
      {
        maze(0, i)      = 1;
        maze(mx - 1, i) = 1;
      }
    }

    std::cout << maze << std::endl;

    for (int i = 0; i < mx; ++i)
    {
      for (int j = 0; j < my; ++j)
      {
        if (maze(i, j))
        {
          for (int ii = 0; ii < width * info.scale; ++ii)
          {
            for (int jj = 0; jj < width * info.scale; ++jj)
            {
              for (int k = 0; k < info.sizeZ; ++k)
              {
                pcl::PointXYZ pt_random;
                pt_random.x =
                  i * width + ii / info.scale - info.sizeX / (2.0 * info.scale);
                pt_random.y =
                  j * width + jj / info.scale - info.sizeY / (2.0 * info.scale);
                pt_random.z = k / info.scale;
                info.cloud->points.push_back(pt_random);
              }
            }
          }
        }
      }
    }
    std::cout << mx <<std::endl;
    std::cout << my <<std::endl;

    info.cloud->width    = info.cloud->points.size();
    info.cloud->height   = 1;
    info.cloud->is_dense = true;
    pcl2ros();

  }

  void Maze3DGen()
  {
    // getting required info parameters from the given node
    int    numNodes;
    double connectivity;
    int    nodeRad;
    int    roadRad;

    this->get_parameter_or("numNodes", numNodes, 10);
    this->get_parameter_or("connectivity", connectivity, 0.5);
    this->get_parameter_or("nodeRad", nodeRad, 3);
    this->get_parameter_or("roadRad", roadRad, 2);
    RCLCPP_INFO(this->get_logger(),
                "received parameters : numNodes: %d connectivity: "
                "%f nodeRad: %d roadRad: %d",
                numNodes,
                connectivity,
                nodeRad,
                roadRad);
    // generating random points
    std::vector<pcl::PointXYZ> base;

    for (int i = 0; i < numNodes; i++)
    {
      double rx = std::rand() / RAND_MAX +
                  (std::rand() % info.sizeX) / info.scale -
                  info.sizeX / (2 * info.scale);
      double ry = std::rand() / RAND_MAX +
                  (std::rand() % info.sizeY) / info.scale -
                  info.sizeY / (2 * info.scale);
      double rz = std::rand() / RAND_MAX +
                  (std::rand() % info.sizeZ) / info.scale -
                  info.sizeZ / (2 * info.scale);
      RCLCPP_INFO(this->get_logger(), "point: x: %f , y: %f , z: %f", rx, ry, rz);

      pcl::PointXYZ pt_random;
      pt_random.x = rx;
      pt_random.y = ry;
      pt_random.z = rz;
      base.push_back(pt_random);
    } // generating random cores in the space

    for (int i = 0; i < info.sizeX; i++)
    {
      for (int j = 0; j < info.sizeY; j++)
      {
        for (int k = 0; k < info.sizeZ; k++)
        { // for every scaled coordinate points
          pcl::PointXYZ test;
          test.x = i / info.scale - info.sizeX / (2 * info.scale);
          test.y = j / info.scale - info.sizeY / (2 * info.scale);
          test.z = k / info.scale -
                  info.sizeZ /
                    (2 * info.scale); // marking the corresponding point location

          MazePoint mp;
          mp.setPoint(test);
          mp.setPoint2(-1);
          mp.setPoint1(-1);
          mp.setDist1(10000.0);
          mp.setDist2(100000.0); // setting super large starting values
          for (int ii = 0; ii < numNodes; ii++)
          {
            double dist =
              std::sqrt((base[ii].x - test.x) * (base[ii].x - test.x) +
                        (base[ii].y - test.y) * (base[ii].y - test.y) +
                        (base[ii].z - test.z) * (base[ii].z - test.z));
            if (dist < mp.getDist1())
            {

              mp.setDist2(mp.getDist1());
              mp.setDist1(dist);

              mp.setPoint2(mp.getPoint1());
              mp.setPoint1(ii);
            }
            else if (dist < mp.getDist2())
            {
              mp.setDist2(dist);
              mp.setPoint2(ii);
            } // finding the distances to the nearest two cores
          }
          if (std::abs(mp.getDist2() - mp.getDist1()) < 1 / info.scale)
          { // the tested location is on one of the middle planes
            if ((mp.getPoint1() + mp.getPoint2()) >
                  int((1 - connectivity) * numNodes) &&
                (mp.getPoint1() + mp.getPoint2()) <
                  int((1 + connectivity) * numNodes))
            { // this is a holed wall
              double judge =
                std::sqrt((base[mp.getPoint1()].x - base[mp.getPoint2()].x) *
                            (base[mp.getPoint1()].x - base[mp.getPoint2()].x) +
                          (base[mp.getPoint1()].y - base[mp.getPoint2()].y) *
                            (base[mp.getPoint1()].y - base[mp.getPoint2()].y) +
                          (base[mp.getPoint1()].z - base[mp.getPoint2()].z) *
                            (base[mp.getPoint1()].z - base[mp.getPoint2()].z));
              if (mp.getDist1() + mp.getDist2() - judge >=
                  roadRad / (info.scale * 3))
              {
                info.cloud->points.push_back(mp.getPoint());
              }
            }
            else
            {
              info.cloud->points.push_back(mp.getPoint());
            }
          }
        }
      }
    }

    info.cloud->width  = info.cloud->points.size();
    info.cloud->height = 1;
    RCLCPP_INFO(this->get_logger(), "the number of points before optimization is %d", info.cloud->width);
    info.cloud->points.resize(info.cloud->width * info.cloud->height);
    pcl2ros();
  }

  void RandomMapGenerateCylinder()
  {
    //! @todo Generate 2D map with constant height
    double _resolution = 1 / info.scale;
    int    _ObsNum;
    this->get_parameter_or("obstacle_number", _ObsNum, 10);

    // Test the basic logic 10*10*10
    pcl::PointXYZ pt_random;
    info.cloud->points.reserve(1000);
    for (int i = 0; i < 10; i++)
    {
      for (int j = 0; j < 10; j++)
      {
        for (int k = 0; k < 10; k++)
        {
          pt_random.x = i * 0.1;
          pt_random.y = j * 0.1;
          pt_random.z = k * 0.1;
          info.cloud->points.push_back(pt_random);
        }
      }
    }
    
    info.cloud->width = info.cloud->points.size();
    info.cloud->height = 1;
    info.cloud->is_dense = true;
    RCLCPP_INFO(this->get_logger(), "the number of points before optimization is %d", info.cloud->width);
    pcl2ros();
  }

  void recursiveDivision(int xl, int xh, int yl, int yh, Eigen::MatrixXi& maze)
  {
    RCLCPP_INFO(this->get_logger(), 
                "generating maze with width %d and height %d",
                xh - xl + 1, yh - yl + 1);

    if (xl < xh - 3 && yl < yh - 3)
    { // the remaining area is larger than or equal to 5*5, need to add both x
      // wall and y wall
      bool valid = false; // used to judge whether the wall selection is valid
      int  xm    = 0;
      int  ym    = 0;
      RCLCPP_INFO(this->get_logger(), "entered 5*5 mode");
      while (valid == false)
      {
        xm = (std::rand() % (xh - xl - 1) + xl +
              1); // generating random number between xl+1 and xh-1(pointless to
                  // add a wall at the sides)
        ym = (std::rand() % (yh - yl - 1) + yl +
              1); // generating random number between yl+1 and yh-1(pointless to
                  // add a wall at the sides)
        if (xl - 1 >= 0)
        { // there is a point at xl-1,ym
          if (maze(xl - 1, ym) == 0)
          { // this is an opening,need to change random number
            continue;
          }
        }

        else if (xh + 1 <= maze.cols() - 1)
        { // there is a point at xh+1,ym
          if (maze(xh + 1, ym) == 0)
          { // this is an opening,need to change random number
            continue;
          }
        }

        else if (yl - 1 >= 0)
        { // there is a point at xm,yl-1
          if (maze(xm, yl - 1) == 0)
          { // this is an opening,need to change random number
            continue;
          }
        }

        else if (yh + 1 <= maze.rows() - 1)
        { // there is a point at xm,yh+1
          if (maze(xm, yh + 1) == 0)
          { // this is an opening,need to change random number
            continue;
          }
        }

        valid = true;

      } // xm and ym are now the valid coordinate of the center of the wall
      for (int i = xl; i <= xh; i++)
      {
        maze(i, ym) = 1;
      }
      for (int j = yl; j <= yh; j++)
      {
        maze(xm, j) = 1;
      } // adding walls around the center point
      int d1 = std::rand() % (xm - xl) + xl;
      int d2 = std::rand() % (xh - xm) + xm + 1;
      int d3 = std::rand() % (ym - yl) + yl;
      int d4 =
        std::rand() % (yh - ym) + ym + 1; // generating four possible door points

      int decision = std::rand() % 4; // random selection of three doors
      switch (decision)
      {
        case 0:
          maze(d1, ym) = 0;
          maze(d2, ym) = 0;
          maze(xm, d3) = 0;
          break;

        case 1:
          maze(d1, ym) = 0;
          maze(d2, ym) = 0;
          maze(xm, d4) = 0;
          break;

        case 2:
          maze(d2, ym) = 0;
          maze(xm, d3) = 0;
          maze(xm, d4) = 0;
          break;

        case 3:
          maze(d1, ym) = 0;
          maze(xm, d3) = 0;
          maze(xm, d4) = 0;
          break;
      } // the doors are opened for this cell
      if (yl - 1 >= 0)
      {
        if (maze(xm, yl - 1) == 0)
        {
          maze(xm, yl) = 0;
        }
      }

      if (yh + 1 <= maze.rows() - 1)
      {
        if (maze(xm, yh + 1) == 0)
        {
          maze(xm, yh) = 0;
        }
      }

      if (xl - 1 >= 0)
      {
        if (maze(xl - 1, ym) == 0)
        {
          maze(xl, ym) = 0;
        }
      }

      if (xh + 1 <= maze.cols() - 1)
      {
        if (maze(xh + 1, ym) == 0)
        {
          maze(xh, ym) = 0;
        }
      }

      std::cout << maze << std::endl;
      recursiveDivision(xl, xm - 1, yl, ym - 1, maze);
      recursiveDivision(xm + 1, xh, yl, ym - 1, maze);
      recursiveDivision(xl, xm - 1, ym + 1, yh, maze);
      recursiveDivision(xm + 1, xh, ym + 1, yh, maze);

      RCLCPP_INFO(this->get_logger(), "finished generating maze with width %d , height %d", xh - xl + 1, yh - yl + 1);
      
      std::cout << maze << std::endl;
      return;
    } // when the remaining area is larger than or equal to 5*5

    else if (xl < xh - 2 && yl < yh - 2)
    {
      //! @todo
      // this valid here seems redundant
      bool valid     = false; // used to judge whether the wall selection is valid
      
      int  xm        = 0;
      int  ym        = 0;
      int  doorcount = 0;
      xm             = (std::rand() % (xh - xl - 1) + xl +
            1); // generating random number between xl+1 and xh-1(pointless to
                            // add a wall at the sides)
      ym =
        (std::rand() % (yh - yl - 1) + yl +
        1); // generating random number between yl+1 and yh-1(pointless to
            // add a wall at the sides)
            // xm and ym are now the valid coordinate of the center of the wall
      for (int i = xl; i <= xh; i++)
      {
        maze(i, ym) = 1;
      }
      for (int j = yl; j <= yh; j++)
      {
        maze(xm, j) = 1;
      } // adding walls around the center point
      if (yl - 1 >= 0)
      {
        if (maze(xm, yl - 1) == 0)
        {
          maze(xm, yl) = 0;
          doorcount++;
        }
      }

      if (yh + 1 <= maze.rows() - 1)
      {
        if (maze(xm, yh + 1) == 0)
        {
          maze(xm, yh) = 0;
          doorcount++;
        }
      }

      if (xl - 1 >= 0)
      {
        if (maze(xl - 1, ym) == 0)
        {
          maze(xl, ym) = 0;
          doorcount++;
        }
      }

      if (xh + 1 <= maze.cols() - 1)
      {
        if (maze(xh + 1, ym) == 0)
        {
          maze(xh, ym) = 0;
          doorcount++;
        }
      }

      int d1 = std::rand() % (xm - xl) + xl;
      int d2 = std::rand() % (xh - xm) + xm + 1;
      int d3 = std::rand() % (ym - yl) + yl;
      int d4 =
        std::rand() % (yh - ym) + ym + 1; // generating four possible door points

      int decision = std::rand() % 4; // random selection of three doors
      switch (decision)
      {
        case 0:
          maze(d1, ym) = 0;
          maze(d2, ym) = 0;
          maze(xm, d3) = 0;
          break;

        case 1:
          maze(d1, ym) = 0;
          maze(d2, ym) = 0;
          maze(xm, d4) = 0;
          break;

        case 2:
          maze(d2, ym) = 0;
          maze(xm, d3) = 0;
          maze(xm, d4) = 0;
          break;

        case 3:
          maze(d1, ym) = 0;
          maze(xm, d3) = 0;
          maze(xm, d4) = 0;
          break;
      } // the doors are opened for this cell
      std::cout << maze << std::endl;

      RCLCPP_INFO(this->get_logger(), 
                  "finished generating maze with width %d , height %d",
                  xh - xl + 1,
                  yh - yl + 1);
      std::cout << maze << std::endl;
      return;
    }

    else if (xl < xh - 1 && yl < yh - 2)
    { // the case of 3*4+
      RCLCPP_INFO(this->get_logger(), "entered 3*4+ mode");
      int doorcount = 0;
      int ym        = 0;
      for (int i = yl; i <= yh; i++)
      {
        maze(xl + 1, i) = 1;
      } // filling a center wall
      if (yl - 1 >= 0)
      {
        if (maze(xl + 1, yl - 1) == 0)
        {
          maze(xl + 1, yl) = 0;
          doorcount++;
        }
      }
      if (yh + 1 <= maze.rows() - 1)
      {
        if (maze(xl + 1, yh + 1) == 0)
        {
          maze(xl + 1, yh) = 0;
          doorcount++;
        }
      } // opening doors if the wall blocks the old doors
      if (doorcount == 0)
      {
        ym               = std::rand() % (yh - yl + 1) + yl;
        maze(xl + 1, ym) = 0;
      }
    } // the case of 4+*3
    //
    else if (xl < xh - 2 && yl < yh - 1)
    { // the case of 4+*3
      RCLCPP_INFO(this->get_logger(), "entered 4+*3 mode");
      int doorcount = 0;
      int xm        = 0;
      for (int i = xl; i <= xh; i++)
      {
        maze(i, yl + 1) = 1;
      } // filling a center wall
      if (xl - 1 >= 0)
      {
        if (maze(xl - 1, yl + 1) == 0)
        {
          maze(xl, yl + 1) = 0;
          doorcount++;
        }
      }
      if (xh + 1 <= maze.cols() - 1)
      {
        if (maze(xh + 1, yl + 1) == 0)
        {
          maze(xh, yl + 1) = 0;
          doorcount++;
        }
      } // opening doors if the wall blocks the old doors
      if (doorcount == 0)
      {
        xm               = std::rand() % (xh - xl + 1) + xl;
        maze(xm, yl + 1) = 0;
      }
    } // the case of 4+*3

    else if (xl < xh - 1 && yl < yh - 1)
    { // the case of 3*3
      maze(xl + 1, yl + 1) = 1;
      return;
    }
    else
    {
      RCLCPP_INFO(this->get_logger(),
                  "finished generating maze with width %d , height %d",
                  xh - xl + 1,
                  yh - yl + 1);
      return;
    }
  }

  void recursizeDivisionMaze(Eigen::MatrixXi& maze)
  {
    //! @todo all bugs here...
    int sx = maze.rows();
    int sy = maze.cols();

    int px, py;

    if (sx > 5)
      px = (std::rand() % (sx - 3) + 1);
    else
      return;

    if (sy > 5)
      py = (std::rand() % (sy - 3) + 1);
    else
      return;

    RCLCPP_INFO(this->get_logger(),
                "debug %d %d %d %d", 
                sx, sy, px, py);

    int x1, x2, y1, y2;

    if (px != 1)
      x1 = (std::rand() % (px - 1) + 1);
    else
      x1 = 1;

    if ((sx - px - 3) > 0)
      x2 = (std::rand() % (sx - px - 3) + px + 1);
    else
      x2 = px + 1;

    if (py != 1)
      y1 = (std::rand() % (py - 1) + 1);
    else
      y1 = 1;

    if ((sy - py - 3) > 0)
      y2 = (std::rand() % (sy - py - 3) + py + 1);
    else
      y2 = py + 1;
    RCLCPP_INFO(this->get_logger(), 
                "%d %d %d %d", 
                x1, x2, y1, y2);

    if (px != 1 && px != (sx - 2))
    {
      for (int i = 1; i < (sy - 1); ++i)
      {
        if (i != y1 && i != y2)
          maze(px, i) = 1;
      }
    }
    if (py != 1 && py != (sy - 2))
    {
      for (int i = 1; i < (sx - 1); ++i)
      {
        if (i != x1 && i != x2)
          maze(i, py) = 1;
      }
    }
    switch (std::rand() % 4)
    {
      case 0:
        maze(x1, py) = 1;
        break;
      case 1:
        maze(x2, py) = 1;
        break;
      case 2:
        maze(px, y1) = 1;
        break;
      case 3:
        maze(px, y2) = 1;
        break;
    }

    if (px > 2 && py > 2)
    {
      Eigen::MatrixXi sub = maze.block(0, 0, px + 1, py + 1);
      recursizeDivisionMaze(sub);
      maze.block(0, 0, px, py) = sub;
    }
    if (px > 2 && (sy - py - 1) > 2)
    {
      Eigen::MatrixXi sub = maze.block(0, py, px + 1, sy - py);
      recursizeDivisionMaze(sub);
      maze.block(0, py, px + 1, sy - py) = sub;
    }
    if (py > 2 && (sx - px - 1) > 2)
    {
      Eigen::MatrixXi sub = maze.block(px, 0, sx - px, py + 1);
      recursizeDivisionMaze(sub);
      maze.block(px, 0, sx - px, py + 1) = sub;
    }
    if ((sx - px - 1) > 2 && (sy - py - 1) > 2)
    {

      Eigen::MatrixXi sub = maze.block(px, py, sy - px, sy - py);

      recursizeDivisionMaze(sub);
      maze.block(px, py, sy - px, sy - py) = sub;
    }
  }

  bool SafeCompare(int a, uint32_t b)
  {
    // If 'a' is negative, it must be smaller than 'b'.
    if (a < 0) return true;

    // Cast 'a' to uint32_t to safely compare it with 'b'.
    return static_cast<uint32_t>(a) < b;
  }

  void optimizeMap()
  {
      std::vector<int>* temp = new std::vector<int>;

      pcl::KdTreeFLANN<pcl::PointXYZ>     kdtree;
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

      cloud->width  = info.cloud->width;
      cloud->height = info.cloud->height;
      cloud->points.resize(cloud->width * cloud->height);
      
      // comparison of integer expressions of different signedness: ‘int’ and ‘uint32_t’
      // for (int i = 0; i < cloud->width; i++)
      for (int i = 0; this->SafeCompare(i, cloud->width); i++)
      {
          cloud->points[i].x = info.cloud->points[i].x;
          cloud->points[i].y = info.cloud->points[i].y;
          cloud->points[i].z = info.cloud->points[i].z;
      }

      kdtree.setInputCloud(cloud);
      double radius = 1.75 / info.scale; // 1.75 is the rounded up value of sqrt(3)

      // for (int i = 0; i < cloud->width; i++)
      for (int i = 0; this->SafeCompare(i, cloud->width); i++)
      {
          std::vector<int>   pointIdxRadiusSearch;
          std::vector<float> pointRadiusSquaredDistance;

          if (kdtree.radiusSearch(cloud->points[i], radius, pointIdxRadiusSearch,
                                  pointRadiusSquaredDistance) >= 27)
          {
              temp->push_back(i);
          }
      }
      for (int i = temp->size() - 1; i >= 0; i--)
      {
          info.cloud->points.erase(info.cloud->points.begin() + temp->at(i)); // erasing the enclosed points
      }
      info.cloud->width -= temp->size();

      pcl::toROSMsg(*info.cloud, *info.output);
      info.output->header.frame_id = "odom";
      // RCLCPP_INFO(this->get_logger(), "finish: number of points after optimization %d", info.cloud->width);
      delete temp;
      return;
  }

private:
  BasicInfo info;
  // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
  // sensor_msgs::msg::PointCloud2::SharedPtr output;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_pub;
  size_t count_;
};

}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mocka::Maps>());
  rclcpp::shutdown();
  return 0;
}