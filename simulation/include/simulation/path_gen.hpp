#ifndef PATH_GEN_HPP
#define PATH_GEN_HPP

#include "gcopter/geo_utils.hpp"
#include "gcopter/firi.hpp"

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

namespace path_gen
{
    template <typename Map>
    inline double planPath(const Eigen::Matrix3Xd &v,
                           const Eigen::Vector3d &s,
                           const Eigen::Vector3d &g,
                           const Eigen::Vector3d &lb,
                           const Eigen::Vector3d &hb,
                           const Map *mapPtr,
                           const double &timeout,
                           std::vector<Eigen::Vector3d> &p)
    {   
        // initialize state space
        auto space(std::make_shared<ompl::base::RealVectorStateSpace>(3));
        ompl::base::RealVectorBounds bounds(3);
        bounds.setLow(0, 0.0);
        bounds.setHigh(0, hb(0) - lb(0));
        bounds.setLow(1, 0.0);
        bounds.setHigh(1, hb(1) - lb(1));
        bounds.setLow(2, 0.0);
        bounds.setHigh(2, hb(2) - lb(2));
        space->setBounds(bounds);
        
        // important: set validity checker
        auto si(std::make_shared<ompl::base::SpaceInformation>(space));
        si->setStateValidityChecker(
            [&](const ompl::base::State *state)
            {
                const auto *pos = state->as<ompl::base::RealVectorStateSpace::StateType>();
                // voxel width is 0.25
                // assuming the robot is a box with diameter of 0.5
                const Eigen::Vector3d position(lb(0) + (*pos)[0],
                                               lb(1) + (*pos)[1],
                                               lb(2) + (*pos)[2]);
                return mapPtr->query_box(position, v) == 0;
            }
        );
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
        
        // initialize problem definition
        auto pdef(std::make_shared<ompl::base::ProblemDefinition>(si));
        pdef->setStartAndGoalStates(start, goal);
        pdef->setOptimizationObjective(std::make_shared<ompl::base::PathLengthOptimizationObjective>(si));
        // initialize solver
        auto planner(std::make_shared<ompl::geometric::InformedRRTstar>(si));
        planner->setProblemDefinition(pdef);
        planner->setup();
        // planner solving status
        ompl::base::PlannerStatus solved;
        solved = planner->ompl::base::Planner::solve(timeout);

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
}

#endif