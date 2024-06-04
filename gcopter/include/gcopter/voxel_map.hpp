#ifndef VOXEL_MAP_HPP
#define VOXEL_MAP_HPP

#include "voxel_dilater.hpp"
#include <memory>
#include <vector>
#include "Eigen/Eigen"

namespace voxel_map
{
    constexpr uint8_t Unoccupied = 0;
    constexpr uint8_t Occupied = 1;
    constexpr uint8_t Dilated = 2;

    class VoxelMap
    {
    
    public:
        VoxelMap() = default;
        VoxelMap(const Eigen::Vector3i &size,
                 const Eigen::Vector3d &origin,
                 const double &voxScale)
            : mapSize(size),
              o(origin),
              scale(voxScale),
              voxNum(mapSize.prod()),
              step(1, mapSize(0), mapSize(1) * mapSize(0)),
              oc(o + Eigen::Vector3d::Constant(0.5 * scale)),
              bounds((mapSize.array() - 1) * step.array()),
              stepScale(step.cast<double>().cwiseInverse() * scale),
              voxels(voxNum, Unoccupied) {}

    private:
        
        Eigen::Vector3i mapSize; // mapSize the voxel counting width, length, and height of the 3D voxel map
        Eigen::Vector3d o;       // the origin of the map, coordinates represented in the world frame
        double scale;            // the size of a voxel in the world frame
        int voxNum;              // total voxels number
        Eigen::Vector3i step;    // 1, width, width*length
        Eigen::Vector3d oc;      // 
        Eigen::Vector3i bounds;  // [width-1, length-1, height-1]T * [1, width, width*length] 
        Eigen::Vector3d stepScale;
        std::vector<uint8_t> voxels; // voxels : size equals to number of voxels
        std::vector<Eigen::Vector3i> surf; // surf : the surface of the points cloud
    
    public:
        inline Eigen::Vector3i getSize(void) const
        {
            return mapSize;
        }

        inline double getScale(void) const
        {
            return scale;
        }

        inline Eigen::Vector3d getOrigin(void) const
        {
            return o;
        }

        inline Eigen::Vector3d getCorner(void) const
        {
            return mapSize.cast<double>() * scale + o;
        }

        inline const std::vector<uint8_t> &getVoxels(void) const
        {
            return voxels;
        }

        inline void setOccupied(const Eigen::Vector3d &pos)
        {   
            // voxel map uses the pixel representation, a 3D int type matrix
            // scale: the distance of a voxel unit
            // o    : the origin of the map
            const Eigen::Vector3i id = ((pos - o) / scale).cast<int>();
            if (id(0) >= 0 && id(1) >= 0 && id(2) >= 0 &&
                id(0) < mapSize(0) && id(1) < mapSize(1) && id(2) < mapSize(2))
            {   
                // voxels is set as a list
                // id.dot(step) point to the relative position of the voxel in the list
                voxels[id.dot(step)] = Occupied;
            }
        }

        inline void setOccupied(const Eigen::Vector3i &id)
        {
            if (id(0) >= 0 && id(1) >= 0 && id(2) >= 0 &&
                id(0) < mapSize(0) && id(1) < mapSize(1) && id(2) < mapSize(2))
            {
                voxels[id.dot(step)] = Occupied;
            }
        }

        inline void dilate(const int &r)
        {   
            // the purpose of this function:
            if (r <= 0)
            {
                return;
            }
            else
            {
                std::vector<Eigen::Vector3i> lvec, cvec;
                lvec.reserve(voxNum);
                cvec.reserve(voxNum);
                int i, j, k, idx;
                bool check;
                // apply some kind of dilation operation to the specified voxel 
                // based on its coordinates and other parameters.
                // bounds: [width-1, length-1, height-1]T * [1, width, width*length] 
                for (int x = 0; x <= bounds(0); x++)
                {
                    for (int y = 0; y <= bounds(1); y += step(1))
                    {
                        for (int z = 0; z <= bounds(2); z += step(2))
                        {   
                            // [0, 0, 0]
                            // [0, 0, width*length]
                            // [1, width, width*length]
                            // retrieve all the voxels
                            if (voxels[x + y + z] == Occupied)
                            {   
                                // for each occupied voxel, do VOXEL_DILATER
                                // expanding the occupied voxels based on certain criteria.
                                VOXEL_DILATER(i, j, k,
                                              x, y, z,
                                              step(1), step(2),
                                              bounds(0), bounds(1), bounds(2),
                                              check, voxels, idx, Dilated, cvec)
                                // the surface voxel of the original points has been saved to cvec
                            }
                        }
                    }
                }
                
                // r: dialateRadius / width per voxel
                // expand the original surface of r circles
                for (int loop = 1; loop < r; loop++)
                {   
                    // swap the value of cvec and lvec
                    // std::vector<Eigen::Vector3i> : cvec, lvec 
                    std::swap(cvec, lvec);
                    for (const Eigen::Vector3i &id : lvec)
                    // for each dilated voxel in the previous step, saved in the lvec
                    // the cvec has been cleared
                    {
                        VOXEL_DILATER(i, j, k,
                                      id(0), id(1), id(2),
                                      step(1), step(2),
                                      bounds(0), bounds(1), bounds(2),
                                      check, voxels, idx, Dilated, cvec)
                                      // Dilated = 2, will set the dilated voxel to 2
                                      // adds the voxel coordinates (i, j, k) to a container cvec
                                      // cvec only save the most outside voxel boudary
                    }
                    lvec.clear();
                }
                // updates the `surf` variable with the contents of the `cvec` vector, 
                // which presumably represents the final dilated surface.
                // 
                surf = cvec;
            }
        }

        inline void getSurfInBox(const Eigen::Vector3i &center,
                                 const int &halfWidth,
                                 std::vector<Eigen::Vector3d> &points) const
        {
            for (const Eigen::Vector3i &id : surf)
            {
                if (std::abs(id(0) - center(0)) <= halfWidth &&
                    std::abs(id(1) / step(1) - center(1)) <= halfWidth &&
                    std::abs(id(2) / step(2) - center(2)) <= halfWidth)
                {
                    points.push_back(id.cast<double>().cwiseProduct(stepScale) + oc);
                }
            }

            return;
        }

        inline void getSurf(std::vector<Eigen::Vector3d> &points) const
        {   
            // Seems this function convert the inner "surf" to another represention 
            // reserve: allocate memory for at least `n` elements upfront, which can be useful 
            // when you know in advance the approximate number of elements the vector will hold.
            // surf is of type std::vector<Eigen::Vector3i>
            // stepScale if of type std::vector<Eigen::Vector3d>
            points.reserve(surf.size());
            for (const Eigen::Vector3i &id : surf)
            {
                points.push_back(id.cast<double>().cwiseProduct(stepScale) + oc);
            }
            return;
        }

        inline bool query(const Eigen::Vector3d &pos) const
        {
            const Eigen::Vector3i id = ((pos - o) / scale).cast<int>();
            if (id(0) >= 0 && id(1) >= 0 && id(2) >= 0 &&
                id(0) < mapSize(0) && id(1) < mapSize(1) && id(2) < mapSize(2))
            {
                return voxels[id.dot(step)];
            }
            else
            {
                return true;
            }
        }

        inline bool query(const Eigen::Vector3i &id) const
        {
            if (id(0) >= 0 && id(1) >= 0 && id(2) >= 0 &&
                id(0) < mapSize(0) && id(1) < mapSize(1) && id(2) < mapSize(2))
            {
                return voxels[id.dot(step)];
            }
            else
            {
                return true;
            }
        }

        inline Eigen::Vector3d posI2D(const Eigen::Vector3i &id) const
        {
            return id.cast<double>() * scale + oc;
        }

        inline Eigen::Vector3i posD2I(const Eigen::Vector3d &pos) const
        {
            return ((pos - o) / scale).cast<int>();
        }
    };
}

#endif