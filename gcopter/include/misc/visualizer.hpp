#ifndef VISUALIZER_HPP
#define VISUALIZER_HPP

#include "gcopter/trajectory.hpp"
#include "gcopter/quickhull.hpp"
#include "gcopter/geo_utils.hpp"

#include <iostream>
#include <memory>
#include <chrono>
#include <cmath>
#include <Eigen/Eigen>

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

// Visualizer for the planner
class Visualizer
{
public:
    // Visualize the key module frames in the trajectory
    //! @todo modify the function when the module planning function is done
    //! @author added by kiki
    template <int D>
    inline void visualizeModule(const Trajectory<D> & traj,
                                const Eigen::Vector3d & moduleSize,
                                const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr & modulePub)
    {
        // Markers are visual elements that can be used to represent things like points, lines, meshes, text, etc.
        visualization_msgs::msg::Marker moduleMarker;
        // Each marker must have a unique identifier within a Marker message
        // By using unique IDs for markers, you can update or remove 
        // specific markers within a visualization without affecting other markers.
        moduleMarker.id = 0;
        moduleMarker.type = visualization_msgs::msg::Marker::LINE_LIST;
        moduleMarker.header.stamp = rclcpp::Clock().now();
        moduleMarker.header.frame_id = "odom";
        // part of the Pose message used to define the orientation of a marker in 3D space.
        // The `w` component represents the scalar (real) part of the quaternion.
        moduleMarker.pose.orientation.w = 1.00;
        // specify what action should be taken with the marker that is being published. 
        // `ADD` (value = 0): This action adds a new marker to the visualization. 
        // If a marker with the same `id` already exists, it will be replaced by the new marker.
        moduleMarker.action = visualization_msgs::msg::Marker::ADD;
        // specify the namespace for the marker. 
        // organize and manage markers more effectively by grouping them into namespaces
        moduleMarker.ns = "moduleframe";
        // geometry and visual settings
        moduleMarker.color.r = 0.00;
        moduleMarker.color.g = 1.00;
        moduleMarker.color.b = 0.00;
        moduleMarker.color.a = 1.00;
        moduleMarker.scale.x = 0.2; // line width

        // visulization
        // current logic: for each waypoint, show a module whose CoM is the waypoint
        std::vector<std::pair<int, int>> box_edges = 
        {
            {0, 1}, {1, 2}, {2, 3}, {3, 0},
            {4, 5}, {5, 6}, {6, 7}, {7, 4},
            {0, 4}, {1, 5}, {2, 6}, {3, 7}
        };

        //! @todo add diameter to hyper parameters
        // const double diameter = 0.5;
        // Eigen::Vector3d offset = Eigen::Vector3d(diameter, diameter, diameter);
        std::vector<Eigen::Vector3d> moduleGeom(8);
        Eigen::Vector3d coGeom(0.0, 0.0, 0.0);

        if (traj.getPieceNum() > 0)
        {   
            // wps represents the segement points
            // Eigen::Matrix wps = traj.getPositions();
            Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> wps = traj.getPositions();
            for (int i=0; i<wps.cols(); i++)
            {   
                // the CoM of the Module
                coGeom = wps.col(i).head<3>();
                // update the moduleGeom
                moduleGeom[0] = coGeom + Eigen::Vector3d(1, 1, 1).cwiseProduct(moduleSize);
                moduleGeom[1] = coGeom + Eigen::Vector3d(-1, 1, 1).cwiseProduct(moduleSize);
                moduleGeom[2] = coGeom + Eigen::Vector3d(-1, -1, 1).cwiseProduct(moduleSize);
                moduleGeom[3] = coGeom + Eigen::Vector3d(1, -1, 1).cwiseProduct(moduleSize);
                moduleGeom[4] = coGeom + Eigen::Vector3d(1, 1, -1).cwiseProduct(moduleSize);
                moduleGeom[5] = coGeom + Eigen::Vector3d(-1, 1, -1).cwiseProduct(moduleSize);
                moduleGeom[6] = coGeom + Eigen::Vector3d(-1, -1, -1).cwiseProduct(moduleSize);
                moduleGeom[7] = coGeom + Eigen::Vector3d(1, -1, -1).cwiseProduct(moduleSize);
                // add line to moduleMarker
                for (std::pair<int, int> edge : box_edges)
                {
                    geometry_msgs::msg::Point point;
                    point.x = moduleGeom[edge.first](0);
                    point.y = moduleGeom[edge.first](1);
                    point.z = moduleGeom[edge.first](2);
                    moduleMarker.points.push_back(point);
                    point.x = moduleGeom[edge.second](0);
                    point.y = moduleGeom[edge.second](1);
                    point.z = moduleGeom[edge.second](2);
                    moduleMarker.points.push_back(point);
                } 
            }

        }

        modulePub->publish(moduleMarker);
        
        return;
    }

    // // std::vector<Eigen::Vector3d> pc;
    // // visualize polytopes in V-representation
    // //! @todo rename the paramters for clarification
    // //! @author add by kiki
    // inline void visualizeModule(const std::vector<Eigen::Vector3d> &vPoly,
    //                             const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr & edgePub)
    // {
    //     // vPoly
    //     // edgePub
    //     // module is a kind of polytope
    //     // RVIZ support tris for visualization
    //     visualization_msgs::msg::Marker edgeMarker;

    //     // initialize the msg type and other parameters
    //     edgeMarker.id = 0;
    //     edgeMarker.header.stamp = rclcpp::Clock().now();
    //     edgeMarker.header.frame_id = "odom";
    //     edgeMarker.pose.orientation.w = 1.00;
    //     edgeMarker.action = visualization_msgs::msg::Marker::ADD;
    //     edgeMarker.type = visualization_msgs::msg::Marker::LINE_LIST;
    //     edgeMarker.ns = "edge";
    //     edgeMarker.color.r = 0.00;
    //     edgeMarker.color.g = 1.00;
    //     edgeMarker.color.b = 1.00;
    //     edgeMarker.color.a = 1.00;
    //     edgeMarker.scale.x = 0.02; // line width

    //     geometry_msgs::msg::Point point;

    //     std::vector<std::pair<int, int>> box_edges = 
    //     {
    //         {0, 1}, {1, 2}, {2, 3}, {3, 0},
    //         {4, 5}, {5, 6}, {6, 7}, {7, 4},
    //         {0, 4}, {1, 5}, {2, 6}, {3, 7}
    //     };

    //     for (std::pair<int, int> edge : box_edges)
    //     {
    //         point.x = vPoly[edge.first](0);
    //         point.y = vPoly[edge.first](1);
    //         point.z = vPoly[edge.first](2);
    //         edgeMarker.points.push_back(point);
    //         point.x = vPoly[edge.second](0);
    //         point.y = vPoly[edge.second](1);
    //         point.z = vPoly[edge.second](2);
    //         edgeMarker.points.push_back(point);
    //     }

    //     edgePub->publish(edgeMarker);
        
    //     return;
    // }

    // Visualize the trajectory and its front-end path
    template <int D>
    inline void visualize(const Trajectory<D> & traj,
                          const std::vector<Eigen::Vector3d> & route,
                          const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr & routePub,
                          const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr & wayPointsPub,
                          const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr & trajectoryPub)
    {
        visualization_msgs::msg::Marker routeMarker, wayPointsMarker, trajMarker;

        routeMarker.id = 0;
        routeMarker.type = visualization_msgs::msg::Marker::LINE_LIST;
        routeMarker.header.stamp = rclcpp::Clock().now();
        routeMarker.header.frame_id = "odom";
        routeMarker.pose.orientation.w = 1.00;
        routeMarker.action = visualization_msgs::msg::Marker::ADD;
        routeMarker.ns = "route";
        routeMarker.color.r = 1.00;
        routeMarker.color.g = 0.00;
        routeMarker.color.b = 0.00;
        routeMarker.color.a = 1.00;
        routeMarker.scale.x = 0.1;
        
        // wayPointsMarker = routeMarker; // seems redundant
        wayPointsMarker = routeMarker;
        wayPointsMarker.id = -wayPointsMarker.id - 1;
        wayPointsMarker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        wayPointsMarker.ns = "waypoints";
        wayPointsMarker.color.r = 1.00;
        wayPointsMarker.color.g = 0.00;
        wayPointsMarker.color.b = 0.00;
        wayPointsMarker.scale.x = 0.35;
        wayPointsMarker.scale.y = 0.35;
        wayPointsMarker.scale.z = 0.35;

        trajMarker = routeMarker;
        trajMarker.header.frame_id = "odom";
        trajMarker.id = 0;
        trajMarker.ns = "trajectory";
        trajMarker.color.r = 0.00;
        trajMarker.color.g = 0.50;
        trajMarker.color.b = 1.00;
        trajMarker.scale.x = 0.30;

        if (route.size()>0)
        {
            bool first = true;
            Eigen::Vector3d last;
            for (auto it : route)
            {
                if (first)
                {
                    first = false;
                    last = it;
                    continue;
                }
                geometry_msgs::msg::Point point;

                point.x = last(0);
                point.y = last(1);
                point.z = last(2);
                routeMarker.points.push_back(point);
                point.x = it(0);
                point.y = it(1);
                point.z = it(2);
                routeMarker.points.push_back(point);
                last = it;
            }

            routePub->publish(routeMarker);
        }

        if (traj.getPieceNum() > 0)
        {   
            // wps represents the segement points
            Eigen::Matrix wps = traj.getPositions();
            for (int i=0; i<wps.cols(); i++)
            {
                geometry_msgs::msg::Point point;
                point.x = wps.col(i)(0);
                point.y = wps.col(i)(1);
                point.z = wps.col(i)(2);
                wayPointsMarker.points.push_back(point);
            }

            wayPointsPub->publish(wayPointsMarker);
        }

        if (traj.getPieceNum() > 0)
        {
            double T = 0.01;
            Eigen::Vector3d lastX = traj.getPos(0.0);
            for (double t = T; t < traj.getTotalDuration(); t += T)
            {
                geometry_msgs::msg::Point point;
                Eigen::Vector3d X = traj.getPos(t);
                point.x = lastX(0);
                point.y = lastX(1);
                point.z = lastX(2);
                trajMarker.points.push_back(point);
                point.x = X(0);
                point.y = X(1);
                point.z = X(2);
                trajMarker.points.push_back(point);
                lastX = X;
            }
            trajectoryPub->publish(trajMarker);
        }
    }
    
    // Visualize some polytopes in H-representation
    inline void visualizePolytope(const std::vector<Eigen::MatrixX4d> &hPolys,
                                  const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr & meshPub,
                                  const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr & edgePub)
    {
        // Due to the fact that H-representation cannot be directly visualized
        // We first conduct vertex enumeration of them, then apply quickhull
        // to obtain triangle meshs of polyhedra

        // initializing these matrices with 3 rows and 0 columns. 
        // This means that initially, these matrices are empty as they do not contain any columns (i.e., no data). 
        // The number of columns in these matrices can be increased later during 
        // runtime as needed based on the data you want to store in them.
        Eigen::Matrix3Xd mesh(3, 0), curTris(3, 0), oldTris(3, 0);
        for (size_t id=0; id<hPolys.size(); id++)
        {
            oldTris = mesh;
            // an Eigen matrix with a fixed number of rows (3) and a dynamic number of columns
            // colMajor: the matrix should be stored in column-major order in memory
            Eigen::Matrix<double, 3, -1, Eigen::ColMajor> vPoly;
            geo_utils::enumerateVs(hPolys[id], vPoly);

            quickhull::QuickHull<double> tinyQH;
            const auto polyHull = tinyQH.getConvexHull(vPoly.data(), vPoly.cols(), false, true);
            // idxBuffer is the set of multiple "triple vertices"
            // each represents a mesh plane 
            const auto &idxBuffer = polyHull.getIndexBuffer();
            int hNum = idxBuffer.size() / 3;

            // hNum is the number of mesh pieces
            curTris.resize(3, hNum * 3);
            for (int i = 0; i < hNum * 3; i++)
            {
                curTris.col(i) = vPoly.col(idxBuffer[i]);
            }

            mesh.resize(3, oldTris.cols() + curTris.cols());
            mesh.leftCols(oldTris.cols()) = oldTris;
            mesh.rightCols(curTris.cols()) = curTris;
        }

        // RVIZ support tris for visualization
        visualization_msgs::msg::Marker meshMarker, edgeMarker;

        meshMarker.id = 0;
        meshMarker.header.stamp = rclcpp::Clock().now();
        meshMarker.header.frame_id = "odom";
        meshMarker.pose.orientation.w = 1.00;
        meshMarker.action = visualization_msgs::msg::Marker::ADD;
        meshMarker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
        meshMarker.ns = "mesh";
        meshMarker.color.r = 0.00;
        meshMarker.color.g = 0.00;
        meshMarker.color.b = 1.00;
        meshMarker.color.a = 0.15;
        meshMarker.scale.x = 1.0;
        meshMarker.scale.y = 1.0;
        meshMarker.scale.z = 1.0;

        edgeMarker = meshMarker;
        edgeMarker.type = visualization_msgs::msg::Marker::LINE_LIST;
        edgeMarker.ns = "edge";
        edgeMarker.color.r = 0.00;
        edgeMarker.color.g = 1.00;
        edgeMarker.color.b = 1.00;
        edgeMarker.color.a = 1.00;
        edgeMarker.scale.x = 0.02;

        geometry_msgs::msg::Point point;
        
        // access the number of columns in the `mesh` matrix. 
        int ptnum = mesh.cols();
        for (int i=0; i<ptnum; i++)
        {
            point.x = mesh(0, i);
            point.y = mesh(1, i);
            point.z = mesh(2, i);
            meshMarker.points.push_back(point);
        }

        for (int i = 0; i < ptnum / 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                point.x = mesh(0, 3 * i + j);
                point.y = mesh(1, 3 * i + j);
                point.z = mesh(2, 3 * i + j);
                edgeMarker.points.push_back(point);
                point.x = mesh(0, 3 * i + (j + 1) % 3);
                point.y = mesh(1, 3 * i + (j + 1) % 3);
                point.z = mesh(2, 3 * i + (j + 1) % 3);
                edgeMarker.points.push_back(point);
            }
        }

        meshPub->publish(meshMarker);
        edgePub->publish(edgeMarker);

        return;
    }

    // Visaulize all spheres with centers sphs and the same radius
    inline void visualizeSphere(const Eigen::Vector3d &center,
                                const double &radius,
                                rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr &spherePub)
    {
        visualization_msgs::msg::Marker sphereMarkers, sphereDeleter;

        sphereMarkers.id = 0;
        sphereMarkers.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        sphereMarkers.header.stamp = rclcpp::Clock().now();
        sphereMarkers.header.frame_id = "odom";
        sphereMarkers.pose.orientation.w = 1.00;
        sphereMarkers.action = visualization_msgs::msg::Marker::ADD;
        sphereMarkers.ns = "spheres";
        sphereMarkers.color.r = 0.00;
        sphereMarkers.color.g = 0.00;
        sphereMarkers.color.b = 1.00;
        sphereMarkers.color.a = 1.00;
        sphereMarkers.scale.x = radius * 2.0;
        sphereMarkers.scale.y = radius * 2.0;
        sphereMarkers.scale.z = radius * 2.0;

        sphereDeleter = sphereMarkers;
        sphereDeleter.action = visualization_msgs::msg::Marker::DELETE;

        geometry_msgs::msg::Point point;
        point.x = center(0);
        point.y = center(1);
        point.z = center(2);
        sphereMarkers.points.push_back(point);

        spherePub->publish(sphereDeleter);
        spherePub->publish(sphereMarkers);
    }

    inline void visualizeStartGoal(const Eigen::Vector3d &center,
                                   const double &radius,
                                   const int sg,
                                   rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr &spherePub)
    {
        visualization_msgs::msg::Marker sphereMarkers, sphereDeleter;

        sphereMarkers.id = sg;
        sphereMarkers.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        sphereMarkers.header.stamp = rclcpp::Clock().now();
        sphereMarkers.header.frame_id = "odom";
        sphereMarkers.pose.orientation.w = 1.00;
        sphereMarkers.action = visualization_msgs::msg::Marker::ADD;
        sphereMarkers.ns = "StartGoal";
        sphereMarkers.color.r = 1.00;
        sphereMarkers.color.g = 0.00;
        sphereMarkers.color.b = 0.00;
        sphereMarkers.color.a = 1.00;
        sphereMarkers.scale.x = radius * 2.0;
        sphereMarkers.scale.y = radius * 2.0;
        sphereMarkers.scale.z = radius * 2.0;

        sphereDeleter = sphereMarkers;
        sphereDeleter.action = visualization_msgs::msg::Marker::DELETEALL;

        geometry_msgs::msg::Point point;
        point.x = center(0);
        point.y = center(1);
        point.z = center(2);
        sphereMarkers.points.push_back(point);

        if (sg == 0)
        {
            spherePub->publish(sphereDeleter);
            rclcpp::sleep_for(std::chrono::nanoseconds(1));
            sphereMarkers.header.stamp = rclcpp::Clock().now();
        }
        spherePub->publish(sphereMarkers);
    }
};

#endif
