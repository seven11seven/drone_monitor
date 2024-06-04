#ifndef MICO_HPP
#define MICO_HPP

#include <Eigen/Eigen>
#include <cmath>

namespace cranemodule
{
    // this HPP file encode the crane-module dynamics and geometric constriants
    class ModuleMap
    {
    public:
        inline void reset(const Eigen::Matrix<double, 3, 8> &relative_corners)
        {
            rel_cor = relative_corners;
            return;
        }

        inline void geo_forward(const Eigen::Vector3d &geometry_center,
                                Eigen::Matrix<double, 3, 8> &relative_corner)
        {

        }


    private:
        // double mass, grav;
        Eigen::Matrix<double, 3, 8> rel_cor;

    };
}
#endif