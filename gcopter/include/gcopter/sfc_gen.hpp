#ifndef SFC_GEN_HPP
#define SFC_GEN_HPP

#include "geo_utils.hpp"
#include "firi.hpp"

// OMPL is a popular open-source C++ library for motion planning. 
// It provides a wide range of tools and algorithms for solving motion planning problems, 
// particularly in robotic systems and other related fields.
#include <ompl/util/Console.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/DiscreteMotionValidator.h>

#include <deque>
#include <memory>
#include <Eigen/Eigen>

namespace sfc_gen
{
    template <typename Map>
    inline double planPath(const Eigen::Vector3d &s,
                           const Eigen::Vector3d &g,
                           const Eigen::Vector3d &lb,
                           const Eigen::Vector3d &hb,
                           const Map *mapPtr,
                           const double &timeout,
                           std::vector<Eigen::Vector3d> &p)
    {   
        // hb, lb: the points coordinate of the space vertices
        // s, g  : the start state and goal state
        // mapPtr: the pointer to the 3D map (usr defined), a class
        // timeout: the restrict of solving the optimization problem
        // p      : the 
        // state space, bounds, space information, state validity checker, state
        // create a opml space of dimension 3
        auto space(std::make_shared<ompl::base::RealVectorStateSpace>(3));
        // set the bounds of the space
        ompl::base::RealVectorBounds bounds(3);
        bounds.setLow(0, 0.0);
        bounds.setHigh(0, hb(0) - lb(0));
        bounds.setLow(1, 0.0);
        bounds.setHigh(1, hb(1) - lb(1));
        bounds.setLow(2, 0.0);
        bounds.setHigh(2, hb(2) - lb(2));
        space->setBounds(bounds);

        // create space information source
        auto si(std::make_shared<ompl::base::SpaceInformation>(space));
        // responsible for evaluating whether a particular state is 
        // collision-free or satisfies other constraints defined by the user.
        // the syntax: [&](){} is to create a function
        si->setStateValidityChecker(
            [&](const ompl::base::State *state)
            {
                const auto *pos = state->as<ompl::base::RealVectorStateSpace::StateType>();
                const Eigen::Vector3d position(lb(0) + (*pos)[0],
                                               lb(1) + (*pos)[1],
                                               lb(2) + (*pos)[2]);
                return mapPtr->query(position) == 0;
            });
        si->setup();

        ompl::msg::setLogLevel(ompl::msg::LOG_NONE);
        // set the start point and goal point in the state space
        ompl::base::ScopedState<> start(space), goal(space);
        start[0] = s(0) - lb(0);
        start[1] = s(1) - lb(1);
        start[2] = s(2) - lb(2);
        goal[0] = g(0) - lb(0);
        goal[1] = g(1) - lb(1);
        goal[2] = g(2) - lb(2);

        // initialize problem definition based on: 
        // state information, start state, end state, optimization obejective
        auto pdef(std::make_shared<ompl::base::ProblemDefinition>(si));
        pdef->setStartAndGoalStates(start, goal);
        pdef->setOptimizationObjective(std::make_shared<ompl::base::PathLengthOptimizationObjective>(si));
        
        // initialize the optimizer/solver based on: state information, problem definition
        auto planner(std::make_shared<ompl::geometric::InformedRRTstar>(si));
        planner->setProblemDefinition(pdef);
        planner->setup();
        // planner solving status
        ompl::base::PlannerStatus solved;
        solved = planner->ompl::base::Planner::solve(timeout);
        
        // 
        double cost = INFINITY;
        if (solved)
        {
            p.clear();
            // cast the result of `*pdef->getSolutionPath()` to a `const ompl::geometric::PathGeometric&` reference
            // It ensures that the object pointed to by `pdef->getSolutionPath()` is of 
            // type `ompl::geometric::PathGeometric` or a type derived from it.
            // If the cast is successful, the reference is then used to construct
            // a new `ompl::geometric::PathGeometric` object named `path_`
            const ompl::geometric::PathGeometric path_ =
                ompl::geometric::PathGeometric(
                    dynamic_cast<const ompl::geometric::PathGeometric &>(*pdef->getSolutionPath()));
            // It's typically used in situations where you need to perform a downcast
            // in a polymorphic hierarchy and want to ensure type safety at runtime.
            
            // Get the results from ompl planner, and save to p
            for (size_t i = 0; i < path_.getStateCount(); i++)
            {
                const auto state = path_.getState(i)->as<ompl::base::RealVectorStateSpace::StateType>()->values;
                p.emplace_back(lb(0) + state[0], lb(1) + state[1], lb(2) + state[2]);
            }
            // The cost of the results
            cost = pdef->getSolutionPath()->cost(pdef->getOptimizationObjective()).value();
        }

        return cost;
    }
    
    inline void convexCover(const std::vector<Eigen::Vector3d> &path,
                            const std::vector<Eigen::Vector3d> &points,
                            const Eigen::Vector3d &lowCorner,
                            const Eigen::Vector3d &highCorner,
                            const double &progress,
                            const double &range,
                            std::vector<Eigen::MatrixX4d> &hpolys,
                            const double eps = 1.0e-6)
    {   
        // path: the point-wise represented path
        // points: the points of map?
        // lowCorner, highCorner: the origin and corner points
        // progress: the max distance, parameters related to difining the convex hull
        // range:    parameters related to defining the convex hull
        // hpolys:   the sequence of convex hulls
        // search for the convex hulls during the path points hpolys
        hpolys.clear();
        const int n = path.size();
        Eigen::Matrix<double, 6, 4> bd = Eigen::Matrix<double, 6, 4>::Zero();
        bd(0, 0) = 1.0;
        bd(1, 0) = -1.0;
        bd(2, 1) = 1.0;
        bd(3, 1) = -1.0;
        bd(4, 2) = 1.0;
        bd(5, 2) = -1.0;

        Eigen::MatrixX4d hp, gap;
        Eigen::Vector3d a, b = path[0];
        std::vector<Eigen::Vector3d> valid_pc;
        std::vector<Eigen::Vector3d> bs;
        valid_pc.reserve(points.size());
        
        for (int i = 1; i < n;)
        {   
            // progress is the maximum distance between the two adjacent points
            // find the maximum point b to point a 
            a = b;
            if ((a - path[i]).norm() > progress)
            {
                b = (path[i] - a).normalized() * progress + a;
            }
            else
            {
                b = path[i];
                i++;
            }
            // if you are adding elements that are already constructed, `push_back()` is suitable. 
            // If you want to construct elements in place within the vector, `emplace_back()` 
            // is a more efficient choice.
            // bs is ?
            bs.emplace_back(b);
            
            // Eigen::Matrix<double, 6, 4> bd
            // the convex hull should constained by the range of the map
            // bd(0:6, 3) is the bounding box includes a and b
            bd(0, 3) = -std::min(std::max(a(0), b(0)) + range, highCorner(0));
            bd(1, 3) = +std::max(std::min(a(0), b(0)) - range, lowCorner(0));
            bd(2, 3) = -std::min(std::max(a(1), b(1)) + range, highCorner(1));
            bd(3, 3) = +std::max(std::min(a(1), b(1)) - range, lowCorner(1));
            bd(4, 3) = -std::min(std::max(a(2), b(2)) + range, highCorner(2));
            bd(5, 3) = +std::max(std::min(a(2), b(2)) - range, lowCorner(2));

            // valid_pc: std::vector<Eigen::Vector3d>
            valid_pc.clear();
            // for every point in the map surface points
            // if satisfying the 
            for (const Eigen::Vector3d &p : points)
            {   
                //Matrix.leftCols<3>() is a method used to select the leftmost `3` columns of a matrix.
                if ((bd.leftCols<3>() * p + bd.rightCols<1>()).maxCoeff() < 0.0)
                {
                    valid_pc.emplace_back(p);
                }
            }
            Eigen::Map<const Eigen::Matrix<double, 3, -1, Eigen::ColMajor>> pc(valid_pc[0].data(), 3, valid_pc.size());

            // firi is a function related to convex hull 
            firi::firi(bd, pc, a, b, hp);
            
            // Eigen::MatrixX4d hp
            // std::vector<Eigen::MatrixX4d> &hpolys
            if (hpolys.size() != 0)
            {
                const Eigen::Vector4d ah(a(0), a(1), a(2), 1.0);
                if (3 <= ((hp * ah).array() > -eps).cast<int>().sum() +
                             ((hpolys.back() * ah).array() > -eps).cast<int>().sum())
                {
                    firi::firi(bd, pc, a, a, gap, 1);
                    hpolys.emplace_back(gap);
                }
            }

            hpolys.emplace_back(hp);
        }
    }

    inline void shortCut(std::vector<Eigen::MatrixX4d> &hpolys)
    {
        // hpolys is the sequence of the convex hulls for a selected path and related point cloud surface
        std::vector<Eigen::MatrixX4d> htemp = hpolys;
        if (htemp.size() == 1)
        {   
            // copy the only hull and insert to the first place.
            // front(): used to access the first element (the element at the beginning) of the vector
            Eigen::MatrixX4d headPoly = htemp.front();
            // insert(): insert a single element or a range of elements into the vector.
            // begin() : obtain an iterator that points to the beginning of the vector.
            htemp.insert(htemp.begin(), headPoly);
        }
        hpolys.clear();

        int M = htemp.size();
        Eigen::MatrixX4d hPoly;
        bool overlap;
        // `std::deque` is a standard library container in C++ 
        // that stands for "double-ended queue". It is a sequence container that allows 
        // for efficient insertion and deletion of elements at both the beginning 
        // and the end of the sequence.
        // similar to a std::vector.  
        std::deque<int> idices;
        idices.push_front(M - 1);
        // ergotic all the convex hulls and find thr overlap between two
        // if overlap, short cut the hulls in between
        for (int i = M - 1; i >= 0; i--)
        {
            for (int j = 0; j < i; j++)
            {
                if (j < i - 1)
                {
                    overlap = geo_utils::overlap(htemp[i], htemp[j], 0.01);
                }
                else
                {
                    overlap = true;
                }
                if (overlap)
                {
                    idices.push_front(j);
                    i = j + 1;
                    break;
                }
            }
        }
        for (const auto &ele : idices)
        {
            hpolys.push_back(htemp[ele]);
        }
    }
}

#endif