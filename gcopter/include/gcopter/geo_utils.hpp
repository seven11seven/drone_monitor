#ifndef GEO_UTILS_HPP
#define GEO_UTILS_HPP

#include "quickhull.hpp"
#include "sdlp.hpp"

#include <Eigen/Eigen>
#include <cfloat>
#include <cstdint>
#include <set>
#include <chrono>

namespace geo_utils
{
    inline bool findInterior(const Eigen::MatrixX4d &hPoly,
                             Eigen::Vector3d &interior)
    {   
        // the logic of this function
        // m is the number of surfaces
        const int m=hPoly.rows();
        
        Eigen::MatrixX4d A(m, 4);
        Eigen::VectorXd b(m);
        Eigen::Vector4d c, x;
        
        // normalize h0, h1, and h2
        const Eigen::ArrayXd hNorm = hPoly.leftCols<3>().rowwise().norm();
        // A(m, :3) is the normalized (_h0, _h1, _h2; ...)
        A.leftCols<3>() = hPoly.leftCols<3>().array().colwise() / hNorm;
        // A(m, :) = (_h0, _h1, _h2, 1; ...)
        A.rightCols<1>().setConstant(1.0);
        // b = (- _h3; ...)
        b = -hPoly.rightCols<1>().array() / hNorm;
        // c = (0; 0; 0; -1)
        c.setZero();
        c(3) = -1.0;
        
        // min cTx, s.t. Ax<=b
        // dim(x) << dim(b)
        // max x4
        // [_h0, _h1, _h2] [x1, x2, x3] + [_h3] <= -x4
        // a inner point in the polytope should be negative
        // a small -x4 means the point is far from the surface
        // so this indeed to find the point in the "centre" of the polytope
        const double minmaxsd = sdlp::linprog<4>(c, A, b, x);
        interior = x.head<3>();

        return minmaxsd < 0.0 && !std::isinf(minmaxsd);
    }

    //! @todo this may be important for some methods
    // check whether the overlap between two polytope exceeds eps (the inner point to the nearest surface)
    inline bool overlap(const Eigen::MatrixX4d &hPoly0,
                        const Eigen::MatrixX4d &hPoly1,
                        const double eps = 1.0e-6)

    {
        const int m = hPoly0.rows();
        const int n = hPoly1.rows();
        Eigen::MatrixX4d A(m + n, 4);
        Eigen::Vector4d c, x;
        Eigen::VectorXd b(m + n);
        A.leftCols<3>().topRows(m) = hPoly0.leftCols<3>();
        A.leftCols<3>().bottomRows(n) = hPoly1.leftCols<3>();
        A.rightCols<1>().setConstant(1.0);
        b.topRows(m) = -hPoly0.rightCols<1>();
        b.bottomRows(n) = -hPoly1.rightCols<1>();
        c.setZero();
        c(3) = -1.0;

        const double minmaxsd = sdlp::linprog<4>(c, A, b, x);

        return minmaxsd < -eps && !std::isinf(minmaxsd);
    }

    struct filterLess
    {
        inline bool operator()(const Eigen::Vector3d &l,
                               const Eigen::Vector3d &r)
        const {
            return l(0) < r(0) ||
                   (l(0) == r(0) &&
                    (l(1) < r(1) ||
                     (l(1) == r(1) &&
                      l(2) < r(2))));
        }
    };

    inline void filterVs(const Eigen::Matrix3Xd &rV,
                         const double &epsilon,
                         Eigen::Matrix3Xd &fV)
    {
        const double mag = std::max(fabs(rV.maxCoeff()), fabs(rV.minCoeff()));
        const double res = mag * std::max(fabs(epsilon) / mag, DBL_EPSILON);
        std::set<Eigen::Vector3d, filterLess> filter;
        fV = rV;
        int offset = 0;
        Eigen::Vector3d quanti;
        for (int i = 0; i < rV.cols(); i++)
        {
            quanti = (rV.col(i) / res).array().round();
            if (filter.find(quanti) == filter.end())
            {
                filter.insert(quanti);
                fV.col(offset) = rV.col(i);
                offset++;
            }
        }
        fV = fV.leftCols(offset).eval();
        return;
    }

    // Each row of hPoly is defined by h0, h1, h2, h3 as
    // h0*x + h1*y + h2*z + h3 <= 0
    // proposed epsilon is 1.0e-6
    inline void enumerateVs(const Eigen::MatrixX4d &hPoly,
                            const Eigen::Vector3d &inner,
                            Eigen::Matrix3Xd &vPoly,
                            const double epsilon = 1.0e-6)
    {
        const Eigen::VectorXd b = -hPoly.rightCols<1>() - hPoly.leftCols<3>() * inner;
        const Eigen::Matrix<double, 3, -1, Eigen::ColMajor> A =
            (hPoly.leftCols<3>().array().colwise() / b.array()).transpose();

        quickhull::QuickHull<double> qh;
        const double qhullEps = std::min(epsilon, quickhull::defaultEps<double>());
        // CCW is false because the normal in quickhull towards interior
        const auto cvxHull = qh.getConvexHull(A.data(), A.cols(), false, true, qhullEps);
        const auto &idBuffer = cvxHull.getIndexBuffer();
        const int hNum = idBuffer.size() / 3;
        Eigen::Matrix3Xd rV(3, hNum);
        Eigen::Vector3d normal, point, edge0, edge1;
        for (int i = 0; i < hNum; i++)
        {
            point = A.col(idBuffer[3 * i + 1]);
            edge0 = point - A.col(idBuffer[3 * i]);
            edge1 = A.col(idBuffer[3 * i + 2]) - point;
            normal = edge0.cross(edge1); //cross in CW gives an outter normal
            rV.col(i) = normal / normal.dot(point);
        }
        filterVs(rV, epsilon, vPoly);
        vPoly = (vPoly.array().colwise() + inner.array()).eval();
        return;
    }

    // Each row of hPoly is defined by h0, h1, h2, h3 as
    // h0*x + h1*y + h2*z + h3 <= 0
    // proposed epsilon is 1.0e-6
    inline bool enumerateVs(const Eigen::MatrixX4d &hPoly,
                            Eigen::Matrix3Xd &vPoly,
                            const double epsilon = 1.0e-6)
    {
        Eigen::Vector3d inner;
        if (findInterior(hPoly, inner))
        {
            enumerateVs(hPoly, inner, vPoly, epsilon);
            return true;
        }
        else
        {
            return false;
        }
    }

}

#endif