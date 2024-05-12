#include "gcopter/sfc_gen.hpp"
#include "gcopter/voxel_map.hpp"

#include <cmath>
#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include <chrono>
#include <random>

class GlobalPlanner
{
public:
    GlobalPlanner(){}

    inline void plan()
    {
        if (startGoal.size() == 2)
        {
            std::vector<Eigen::Vector3d> route;
            
            // >>> TODO Debug >>>
            sfc_gen::planPath<voxel_map::VoxelMap>(startGoal[0],
                                                   startGoal[1],
                                                   voxelMap.getOrigin(),
                                                   voxelMap.getCorner(),
                                                   &voxelMap, 0.01,
                                                   route);
            // <<< TODO Debug <<<
        }
    }


private:
    // Visualizer visualizer;
    std::string mapTopic;
    std::string targetTopic;
    double dilateRadius;
    double voxelWidth;
    std::vector<double> mapBound;
    double timeoutRRT;
    double maxVelMag;
    double maxBdrMag;
    double maxTiltAngle;
    double minThrust;
    double maxThrust;
    double vehicleMass;
    double gravAcc;
    double horizDrag;
    double vertDrag;
    double parasDrag;
    double speedEps;
    double weightT;
    std::vector<double> chiVec;
    double smoothingEps;
    int integralIntervs;
    double relCostTol;


    bool mapInitialized;
    voxel_map::VoxelMap voxelMap;
    std::vector<Eigen::Vector3d> startGoal;
    
    double trajStamp;
};

int main(int argc, char const *argv[])
{
    /* code */
    GlobalPlanner global_planning = GlobalPlanner();
    global_planning.plan();
    return 0;
}
