#include "misc/visualizer.hpp"
#include "gcopter/trajectory.hpp"
#include "gcopter/gcopter.hpp"
#include "gcopter/firi.hpp"
#include "gcopter/flatness.hpp"
#include "gcopter/voxel_map.hpp"
#include "gcopter/sfc_gen.hpp"

#include "simulation/path_gen.hpp"
#include "simulation/modular.hpp"

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
    mapInitialized(false)
    {   
        this->declare_parameter<std::string>("MapTopic");
        this->declare_parameter<std::string>("TargetTopic");
        this->declare_parameter<double>("DilateRadius");
        this->declare_parameter<double>("VoxelWidth");
        this->declare_parameter<std::vector<double>>("MapBound");
        this->declare_parameter<double>("TimeoutRRT");
        this->declare_parameter<double>("MaxVelMag");
        this->declare_parameter<double>("MaxJibOmg");
        this->declare_parameter<double>("MaxAccMag");
        this->declare_parameter<double>("MinThrust");
        this->declare_parameter<double>("MaxThrust");
        
        this->declare_parameter<double>("ModuleMass");
        this->declare_parameter<double>("JibMass");
        this->declare_parameter<double>("TrolleyMass");
        this->declare_parameter<double>("JibInnt");
        this->declare_parameter<double>("TrolleyInnt");
        this->declare_parameter<double>("GravAcc");
        this->declare_parameter<double>("X");
        this->declare_parameter<double>("Y");
        this->declare_parameter<double>("H");
        this->declare_parameter<double>("L");
        
        this->declare_parameter<double>("WeightT");
        this->declare_parameter<std::vector<double>>("ChiVec");
        this->declare_parameter<double>("SmoothingEps");
        this->declare_parameter<int>("IntegralIntervs");
        this->declare_parameter<double>("RelCostTol");

        this->get_parameter("MapTopic", mapTopic);
        this->get_parameter("TargetTopic", targetTopic);
        this->get_parameter("DilateRadius", dilateRadius);
        this->get_parameter("VoxelWidth", voxelWidth);
        this->get_parameter("MapBound", mapBound);
        this->get_parameter("TimeoutRRT", timeoutRRT);
        this->get_parameter("MaxVelMag", maxVelMag);
        this->get_parameter("MaxJibOmg", maxJibOmg);
        this->get_parameter("MaxAccMag", maxAccMag);
        this->get_parameter("MinThrust", minThrust);
        this->get_parameter("MaxThrust", maxThrust);

        this->get_parameter("ModuleMass", moduleMass);
        this->get_parameter("JibMass", jibMass);
        this->get_parameter("TrolleyMass", trolleyMass);
        this->get_parameter("JibInnt", jibInnt);
        this->get_parameter("TrolleyInnt", trolleyInnt);
        this->get_parameter("GravAcc", gravAcc);
        this->get_parameter("X", X);
        this->get_parameter("Y", Y);
        this->get_parameter("H", H);
        this->get_parameter("L", L);

        this->get_parameter("WeightT", weightT);
        this->get_parameter("ChiVec", chiVec);
        this->get_parameter("SmoothingEps", smoothingEps);
        this->get_parameter("IntegralIntervs", integralIntervs);
        this->get_parameter("RelCostTol", relCostTol);

        visualizer = Visualizer();
        routePub = this->create_publisher<visualization_msgs::msg::Marker>("/visualizer/route", 10);
        wayPointsPub = this->create_publisher<visualization_msgs::msg::Marker>("/visualizer/waypoints", 10);
        trajectoryPub = this->create_publisher<visualization_msgs::msg::Marker>("/visualizer/trajectory", 10);
        meshPub = this->create_publisher<visualization_msgs::msg::Marker>("/visualizer/mesh", 1000);
        edgePub = this->create_publisher<visualization_msgs::msg::Marker>("/visualizer/edge", 1000);
        spherePub = this->create_publisher<visualization_msgs::msg::Marker>("/visualizer/spheres", 1000);
        speedPub = this->create_publisher<std_msgs::msg::Float64>("/visualizer/speed", 1000);
        thrPub = this->create_publisher<std_msgs::msg::Float64>("/visualizer/total_thrust", 1000);
        tiltPub = this->create_publisher<std_msgs::msg::Float64>("/visualizer/tilt_angle", 1000);
        bdrPub = this->create_publisher<std_msgs::msg::Float64>("/visualizer/body_rate", 1000);
        modulePub = this->create_publisher<visualization_msgs::msg::Marker>("/visualizer/modules", 1000);
 
        const Eigen::Vector3i xyz((mapBound[1] - mapBound[0]) / voxelWidth,
                                  (mapBound[3] - mapBound[2]) / voxelWidth,
                                  (mapBound[5] - mapBound[4]) / voxelWidth);

        const Eigen::Vector3d offset(mapBound[0], mapBound[2], mapBound[4]);

        voxelMap = voxel_map::VoxelMap(xyz, offset, voxelWidth);

        mapSub = this->create_subscription<sensor_msgs::msg::PointCloud2>(mapTopic,
                                                                          rclcpp::QoS(1).best_effort(),
                                                                          std::bind(&MiCModule::mapCallBack, this, std::placeholders::_1));

        targetSub = this->create_subscription<geometry_msgs::msg::PoseStamped>(targetTopic,
                                                                               rclcpp::QoS(1).best_effort(),
                                                                               std::bind(&MiCModule::targetCallBack, this, std::placeholders::_1));

    }

    inline void mapCallBack(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {   
        // Convert the PC2 msg to the voxelMap
        // initialize the voxel map if not
        if (!mapInitialized)
        {
            size_t cur = 0;
            // The `point_step` field in `sensor_msgs::msg::PointCloud2` 
            // represents the size, in bytes, of an individual point in the point cloud data.
            // size_t is the number of points in the PCL msg
            const size_t total = msg->data.size() / msg->point_step;
            // fdata is the sequence of float number whose address pointing to the data's address
            float *fdata = (float *)(&msg->data[0]);
            for (size_t i = 0; i < total; i++)
            {   
                // set the points to voxelMap
                cur = msg->point_step / sizeof(float) * i;

                if (std::isnan(fdata[cur + 0]) || std::isinf(fdata[cur + 0]) ||
                    std::isnan(fdata[cur + 1]) || std::isinf(fdata[cur + 1]) ||
                    std::isnan(fdata[cur + 2]) || std::isinf(fdata[cur + 2]))
                {
                    continue;
                }
                voxelMap.setOccupied(Eigen::Vector3d(fdata[cur + 0],
                                                     fdata[cur + 1],
                                                     fdata[cur + 2]));
            }
            // dilateRadius is a predifined paramter
            // voxelScale is const and defined through voxelWidth 
            // dilate the voxel in the point cloud
            // surf is generated at this step
            voxelMap.dilate(std::ceil(dilateRadius / voxelMap.getScale()));

            mapInitialized = true;
        }
    }

    inline void module_plan()
    {
        if (startGoal.size() == 2)
        {
            std::vector<Eigen::Vector3d> route;

            // return the planned path points in vector 3D format
            // planner: RRTstar
            path_gen::planPath<voxel_map::VoxelMap>(startGoal[0],
                                                    startGoal[1],
                                                    voxelMap.getOrigin(),
                                                    voxelMap.getCorner(),
                                                    &voxelMap, 0.01,
                                                    route);
            std::vector<Eigen::MatrixX4d> hPolys;
            std::vector<Eigen::Vector3d> pc;
            voxelMap.getSurf(pc);

            // RCLCPP_WARN(this->get_logger(), "debug point 1");

            sfc_gen::convexCover(route,
                                 pc,
                                 voxelMap.getOrigin(),
                                 voxelMap.getCorner(),
                                 7.0,
                                 3.0,
                                 hPolys);
            sfc_gen::shortCut(hPolys);

            // RCLCPP_WARN(this->get_logger(), "the number of hPolys is %ld", hPolys.size());

            if (route.size() > 1)
            {
                visualizer.visualizePolytope(hPolys, this->meshPub, this->edgePub);

                // state here includes pos, vel, and acc
                Eigen::Matrix3d iniState;
                Eigen::Matrix3d finState;
                iniState << route.front(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();
                finState << route.back(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();

                gcopter::GCOPTER_PolytopeSFC gcopter;
                modular::MODULAR_PolytopeSFC modular;

                //! @todo when modifying the module dynamics
                // initialize some constraint parameters
                Eigen::VectorXd magnitudeBounds(5);
                Eigen::VectorXd penaltyWeights(5);
                Eigen::VectorXd physicalParams(10);

                magnitudeBounds(0) = maxVelMag;
                magnitudeBounds(1) = maxJibOmg;
                magnitudeBounds(2) = maxAccMag;
                magnitudeBounds(3) = minThrust;    //! not used currently
                magnitudeBounds(4) = maxThrust;    //! not used currently
                
                penaltyWeights(0) = (chiVec)[0];
                penaltyWeights(1) = (chiVec)[1];
                penaltyWeights(2) = (chiVec)[2];
                penaltyWeights(3) = (chiVec)[3];
                penaltyWeights(4) = (chiVec)[4];    //! not used currently

                physicalParams(0) = moduleMass;
                physicalParams(1) = jibMass;
                physicalParams(2) = trolleyMass;
                physicalParams(3) = jibInnt;
                physicalParams(4) = trolleyInnt;
                physicalParams(5) = gravAcc;
                physicalParams(6) = X;
                physicalParams(7) = Y;
                physicalParams(8) = H;
                physicalParams(9) = L;

                
                const int quadratureRes = integralIntervs;

                traj.clear();

                if (!modular.setup(weightT,
                                   iniState, finState,
                                   hPolys, INFINITY,
                                   smoothingEps,
                                   quadratureRes,
                                   magnitudeBounds,
                                   penaltyWeights,
                                   physicalParams))
                {
                    return;
                }

                if (std::isinf(modular.optimize(traj, relCostTol)))
                {
                    return;
                }

                if (traj.getPieceNum() > 0)
                {
                    // get the trajectory generation time
                    trajStamp = rclcpp::Clock().now().seconds();
                    visualizer.visualize(traj, route, this->routePub, this->wayPointsPub, this->trajectoryPub);
                    visualizer.visualizeModule(traj, this->modulePub);
                }
            }
        }
    }

    inline void targetCallBack(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        if (mapInitialized)
        {
            if (startGoal.size() >= 2)
            {
                startGoal.clear();
            }
            // bottom level + safe distance + orientation * (height - 2*safe distance)
            // range: [bottom+safe distance, top-safe distance]
            const double zGoal = mapBound[4] + dilateRadius +
                                 fabs(msg->pose.orientation.z) *
                                     (mapBound[5] - mapBound[4] - 2 * dilateRadius);
            const Eigen::Vector3d goal(msg->pose.position.x, msg->pose.position.y, zGoal);
            if (voxelMap.query(goal) == 0)
            {
                visualizer.visualizeStartGoal(goal, 0.5, startGoal.size(), this->spherePub);
                startGoal.emplace_back(goal);
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Infeasible Position Selected !!!\n");
            }

            module_plan();
        }
        return;
    }

private:
    Visualizer visualizer;
    std::string mapTopic;
    std::string targetTopic;
    double dilateRadius;
    double voxelWidth;
    std::vector<double> mapBound;
    double timeoutRRT;
    double maxVelMag;
    double maxJibOmg;
    double maxAccMag;
    double minThrust;
    double maxThrust;

    double moduleMass;
    double jibMass;
    double trolleyMass;
    double jibInnt;
    double trolleyInnt;
    double gravAcc;
    double X;
    double Y;
    double H;
    double L;
    
    double weightT;
    std::vector<double> chiVec;
    double smoothingEps;
    int integralIntervs;
    double relCostTol;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr mapSub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr targetSub;

    bool mapInitialized;
    voxel_map::VoxelMap voxelMap;
    std::vector<Eigen::Vector3d> startGoal;

    Trajectory<5> traj;
    double trajStamp;

public:
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr routePub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr wayPointsPub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr trajectoryPub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr meshPub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr edgePub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr spherePub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr modulePub;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr speedPub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr thrPub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr tiltPub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr bdrPub;
};

int main(int argc, char **argv)
{   
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MiCModule>());
    rclcpp::shutdown();
    return 0;
}
