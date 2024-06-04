#ifndef MODULAR_HPP
#define MODULAR_HPP

#include "minco.hpp"
#include "flatness.hpp"
// the flatness should be converted to the dynamics of the crane-module system
#include "lbfgs.hpp"
#include "geo_utils.hpp"

#include <Eigen/Eigen>
#include <cmath>
#include <cfloat>
#include <iostream>
#include <vector>

namespace modular
{
    class MODULAR_PolytopeSFC
    {
    public:
        typedef Eigen::Matrix3Xd PolyhedronV;   // Vertice representation (use vertices)
        typedef Eigen::MatrixX4d PolyhedronH;   // H-represenation (use inqualities)
        typedef std::vector<PolyhedronV> PolyhedraV;
        typedef std::vector<PolyhedronH> PolyhedraH;

    private:
        // minco provides basic mappings and gradient calculation methods 
        // and can be used in modular trajectory planning without modification
        minco::MINCO_S3NU minco;
        // the flatmap should be refined for the crane-module system
        flatness::FlatnessMap flatmap;

        double rho;
        // start and end state of the module
        Eigen::Matrix3d headPVA;
        Eigen::Matrix3d tailPVA;
        
        PolyhedraV vPolytopes;
        PolyhedraH hPolytopes;
        Eigen::Matrix3Xd shortPath;
        
        // the uid list of trajectory piece
        Eigen::VectorXi pieceIdx;
        Eigen::VectorXi vPolyIdx;
        Eigen::VectorXi hPolyIdx;

        int polyN;
        int pieceN;

        int spatialDim;
        int temporalDim;

        double smoothEps;
        int integralRes;
        Eigen::VectorXd magnitudeBd;
        Eigen::VectorXd penaltyWt;
        Eigen::VectorXd physicalPm;
        double allocSpeed;

        // paramters for lbfgs optimiztaion algor.
        lbfgs::lbfgs_parameter_t lbfgs_params;

        Eigen::Matrix3Xd points;
        Eigen::VectorXd times;
        // Refer to the MINCO paper, partial gradient to point and time variables
        Eigen::Matrix3Xd gradByPoints;
        Eigen::VectorXd gradByTimes;
        Eigen::MatrixX3d partialGradByCoeffs;
        Eigen::VectorXd partialGradByTimes;

        // QQ: vPoly representing the module
        // coordinates are related to the CoM of the module
        Eigen::Matrix<double, 3, 8> modulePoly;

    private:
        /* The following 3 functions give the mappings and gradient w.r.t T and tau */
        static inline void forwardT(const Eigen::VectorXd &tau,
                                    Eigen::VectorXd &T)
        {
            // the diffeomprphism between T and tau
            // from tau to T
            const int sizeTau = tau.size();
            T.resize(sizeTau);
            for (int i=0; i<sizeTau; i++)
            {
                T(i) = tau(i) > 0.0
                           ? ((0.5 * tau(i) + 1.0) * tau(i) + 1.0)
                           : 1.0 / ((0.5 * tau(i) - 1.0) * tau(i) + 1.0);
            }
            return;
        }

        template <typename EIGENVEC>
        static inline void backwardT(const Eigen::VectorXd &T,
                                     EIGENVEC &tau)
        {   
            // the diffeomprphism between T and tau
            // from T to tau
            const int sizeT = T.size();
            tau.resize(sizeT);
            for (int i = 0; i < sizeT; i++)
            {
                tau(i) = T(i) > 1.0
                             ? (sqrt(2.0 * T(i) - 1.0) - 1.0)
                             : (1.0 - sqrt(2.0 / T(i) - 1.0));
            }

            return;
        }

        template <typename EIGENVEC>
        static inline void backwardGradT(const Eigen::VectorXd &tau,
                                         const Eigen::VectorXd &gradT,
                                         EIGENVEC &gradTau)
        {   
            // The backwardGrad can be divided into multiple pieces
            // gradT: penalty gradient w.r.t. (p, T)
            // gradTau: penalty gradient w.r.t. (xi, tau)
            const int sizeTau = tau.size();
            gradTau.resize(sizeTau);
            double denSqrt;
            for (int i = 0; i < sizeTau; i++)
            {
                if (tau(i) > 0)
                {
                    gradTau(i) = gradT(i) * (tau(i) + 1.0);
                }
                else
                {
                    denSqrt = (0.5 * tau(i) - 1.0) * tau(i) + 1.0;
                    gradTau(i) = gradT(i) * (1.0 - tau(i)) / (denSqrt * denSqrt);
                }
            }

            return;
        }

        /* The following 4 functions give the mappings and gradient w.r.t vPolys and P */
        // the functions shall be reused in the geometry penalty
        static inline void forwardP(const Eigen::VectorXd &xi,
                                    const Eigen::VectorXi &vIdx,
                                    const PolyhedraV &vPolys,
                                    Eigen::Matrix3Xd &P)
        {
            // from xi to P
            const int sizeP = vIdx.size();
            P.resize(3, sizeP);
            Eigen::VectorXd q;
            for (int i = 0, j = 0, k, l; i < sizeP; i++, j += k)
            {
                l = vIdx(i);
                k = vPolys[l].cols();
                q = xi.segment(j, k).normalized().head(k - 1);
                P.col(i) = vPolys[l].rightCols(k - 1) * q.cwiseProduct(q) +
                           vPolys[l].col(0);
            }
            return;
        }

        static inline double costTinyNLS(void *ptr,
                                         const Eigen::VectorXd &xi,
                                         Eigen::VectorXd &gradXi)
        {
            const int n = xi.size();
            // what is ovPoly
            const Eigen::Matrix3Xd &ovPoly = *(Eigen::Matrix3Xd *)ptr;

            const double sqrNormXi = xi.squaredNorm();
            const double invNormXi = 1.0 / sqrt(sqrNormXi);
            const Eigen::VectorXd unitXi = xi * invNormXi;
            const Eigen::VectorXd r = unitXi.head(n - 1);
            const Eigen::Vector3d delta = ovPoly.rightCols(n - 1) * r.cwiseProduct(r) +
                                          ovPoly.col(1) - ovPoly.col(0);

            double cost = delta.squaredNorm();
            gradXi.head(n - 1) = (ovPoly.rightCols(n - 1).transpose() * (2 * delta)).array() *
                                 r.array() * 2.0;
            gradXi(n - 1) = 0.0;
            gradXi = (gradXi - unitXi.dot(gradXi) * unitXi).eval() * invNormXi;

            const double sqrNormViolation = sqrNormXi - 1.0;
            if (sqrNormViolation > 0.0)
            {
                double c = sqrNormViolation * sqrNormViolation;
                const double dc = 3.0 * c;
                c *= sqrNormViolation;
                cost += c;
                gradXi += dc * 2.0 * xi;
            }

            return cost;
        }

        template <typename EIGENVEC>
        static inline void backwardP(const Eigen::Matrix3Xd &P,
                                     const Eigen::VectorXi &vIdx,
                                     const PolyhedraV &vPolys,
                                     EIGENVEC &xi)
        {
            const int sizeP = P.cols();

            double minSqrD;
            lbfgs::lbfgs_parameter_t tiny_nls_params;
            tiny_nls_params.past = 0;
            tiny_nls_params.delta = 1.0e-5;
            tiny_nls_params.g_epsilon = FLT_EPSILON;
            tiny_nls_params.max_iterations = 128;

            Eigen::Matrix3Xd ovPoly;
            for (int i = 0, j = 0, k, l; i < sizeP; i++, j += k)
            {
                l = vIdx(i);
                k = vPolys[l].cols();

                ovPoly.resize(3, k + 1);
                ovPoly.col(0) = P.col(i);
                ovPoly.rightCols(k) = vPolys[l];
                Eigen::VectorXd x(k);
                x.setConstant(sqrt(1.0 / k));
                lbfgs::lbfgs_optimize(x,
                                      minSqrD,
                                      &GCOPTER_PolytopeSFC::costTinyNLS,
                                      nullptr,
                                      nullptr,
                                      &ovPoly,
                                      tiny_nls_params);

                xi.segment(j, k) = x;
            }

            return;
        }

        template <typename EIGENVEC>
        static inline void backwardGradP(const Eigen::VectorXd &xi,
                                         const Eigen::VectorXi &vIdx,
                                         const PolyhedraV &vPolys,
                                         const Eigen::Matrix3Xd &gradP,
                                         EIGENVEC &gradXi)
        {
            const int sizeP = vIdx.size();
            gradXi.resize(xi.size());

            double normInv;
            Eigen::VectorXd q, gradQ, unitQ;
            for (int i = 0, j = 0, k, l; i < sizeP; i++, j += k)
            {
                l = vIdx(i);
                k = vPolys[l].cols();
                q = xi.segment(j, k);
                normInv = 1.0 / q.norm();
                unitQ = q * normInv;
                gradQ.resize(k);
                gradQ.head(k - 1) = (vPolys[l].rightCols(k - 1).transpose() * gradP.col(i)).array() *
                                    unitQ.head(k - 1).array() * 2.0;
                gradQ(k - 1) = 0.0;
                gradXi.segment(j, k) = (gradQ - unitQ * unitQ.dot(gradQ)) * normInv;
            }

            return;
        }

        /* Space  */
        template <typename EIGENVEC>
        static inline void normRetrictionLayer(const Eigen::VectorXd &xi,
                                               const Eigen::VectorXi &vIdx,
                                               const PolyhedraV &vPolys,
                                               double &cost,
                                               EIGENVEC &gradXi)
        {
            // from xi to vPolys, spactial optimization
            // xi: the diffeomprphism space to Poly
            // vIdx: the uid list of vPolys
            // vPolys: the v-Polys list
            // cost: xi is supposed in a ball, if not there is a cost
            // gradXi: d(cost) / d(xi) 
            const int sizeP = vIdx.size();
            gradXi.resize(xi.size());
            
            double sqrNormQ, sqrNormViolation, c, dc;
            Eigen::VectorXd q;
            
            for (int i=0, j=0, k; i<sizeP; i++, j += k)
            {   
                // k is the number of colums in i th vPolys
                k = vPolys[vIdx(i)].cols();
                // xi[j:k] is current vPoly's corresponding xi space
                // k verticies -- k coordinates
                q = xi.segment(j, k);

                sqrNormQ = q.squaredNorm();
                sqrNormViolation = sqrNormQ - 1.0;
                if (sqrNormViolation > 0.0)
                // out of the Poly space; adjust xi thorugh back propagation; 
                {
                    // dc = 3 * sqrNormViolation^2
                    c = sqrNormViolation * sqrNormViolation;
                    dc = 3.0 * c;
                    c *= sqrNormViolation;
                    // cost = sum(sqrNormViolation^3)
                    cost += c;
                    // gradXi = d(sqrNormViolation^3) / d(xi)
                    // note that there are many other parts related to xi
                    // they should be added
                    gradXi.segment(j, k) += dc * 2.0 * q;
                }
            }
            return;
        }

        // a basic function to update parameters through gradients
        static inline bool smoothedL1(const double &x,
                                      const double &mu,
                                      double &f,
                                      double &df)
        {   
            // x is the violation degree
            // mu is the threshold, say smoothFactor
            // if x > mu, the penalty is (x-0.5mu), the gradient is 1;
            // if 0< x < mu, the penalty is (mu-0.5x)(x/mu)^3, the gradient is XXX;
            if (x<0.0)
            {
                return false;
            }
            else if (x>mu)
            {
                f = x - 0.5*mu;
                df = 1.0;
                return true;
            }
            else // x in [0, mu]
            {
                const double xdmu = x / mu;
                const double sqrxdmu = xdmu * xdmu;
                const double mumxd2 = mu - 0.5 * x;
                f = mumxd2 * sqrxdmu * xdmu;
                df = sqrxdmu * ((-0.5) * xdmu + 3.0 * mumxd2 / mu);
                return true;
            }
        }

        /* This function calculate the gradient from user defined penalty w.r.t. C and T*/
        //! @todo modify from here. add penalties of the modular existing the Polytope;
        // you may paint an gradient graph to illustrate the process
        // magnitudeBounds = [v_max, omg_max, theta_max, thrust_min, thrust_max]^T
        // penaltyWeights = [pos_weight, vel_weight, omg_weight, theta_weight, thrust_weight]^T
        // physicalParams = [vehicle_mass, gravitational_acceleration, horitonral_drag_coeff,
        //                   vertical_drag_coeff, parasitic_drag_coeff, speed_smooth_factor]^T
        static inline void attachPenaltyFunctional(const Eigen::VectorXd &T,
                                                   const Eigen::MatrixX3d &coeffs,
                                                   const Eigen::VectorXi &hIdx,
                                                   const PolyhedraH &hPolys,
                                                   const double &smoothFactor,
                                                   const int &integralResolution,
                                                   const Eigen::VectorXd &magnitudeBounds,
                                                   const Eigen::VectorXd &penaltyWeights,
                                                   flatness::FlatnessMap &flatMap,
                                                   double &cost,
                                                   Eigen::VectorXd &gradT,
                                                   Eigen::MatrixX3d &gradC)
        {   
            // T: initial trajectory time list
            // magnitudeBounds:
            // smoothFactor is a hyper-parameter to set the penalty
            // this function seems to be the total penalty function
            // aims at minimizing cost through gradT and gradC
            const double velSqrMax = magnitudeBounds(0) * magnitudeBounds(0);
            const double omgSqrMax = magnitudeBounds(1) * magnitudeBounds(1);
            const double thetaMax = magnitudeBounds(2);
            const double thrustMean = 0.5 * (magnitudeBounds(3) + magnitudeBounds(4));
            const double thrustRadi = 0.5 * fabs(magnitudeBounds(4) - magnitudeBounds(3));
            const double thrustSqrRadi = thrustRadi * thrustRadi;

            // position penalty; velocity penalty; thrust penalty;
            // theta penalty: theta, the forth flatoutputs
            // omg penalty:   angular velocity penalty
            const double weightPos = penaltyWeights(0);
            const double weightVel = penaltyWeights(1);
            const double weightOmg = penaltyWeights(2);
            const double weightTheta = penaltyWeights(3);
            const double weightThrust = penaltyWeights(4);

            Eigen::Vector3d pos, vel, acc, jer, sna;
            Eigen::Vector3d totalGradPos, totalGradVel, totalGradAcc, totalGradJer;
            double totalGradPsi, totalGradPsiD;
            double thr, cos_theta;
            // quat: the orientation of the drone, represented in quaterion
            Eigen::Vector4d quat;
            // omg: angular velocity
            Eigen::Vector3d omg;
            double gradThr;
            Eigen::Vector4d gradQuat;
            Eigen::Vector3d gradPos, gradVel, gradOmg;

            double step, alpha;
            double s1, s2, s3, s4, s5;
            Eigen::Matrix<double, 6, 1> beta0, beta1, beta2, beta3, beta4;
            Eigen::Vector3d outerNormal;
            int K, L;
            double violaPos, violaVel, violaOmg, violaTheta, violaThrust;
            double violaPosPenaD, violaVelPenaD, violaOmgPenaD, violaThetaPenaD, violaThrustPenaD;
            double violaPosPena, violaVelPena, violaOmgPena, violaThetaPena, violaThrustPena;
            double node, pena;

            const int pieceNum = T.size();
            const double integralFrac = 1.0 / integralResolution;

            for (int i=0; i<pieceNum; i++) // for each piece in the trajectory
            {
                // In Eigen, the `block` method is used to extract a block of coefficients from a matrix. 
                // The `block` method allows you to select a submatrix from an existing matrix.
                // coeffs means the list of c
                // for one piece in the trajectory, there will be 6*3 coeff paramters
                const Eigen::Matrix<double, 6, 3> &c = coeffs.block<6, 3>(i*6, 0);
                // seems T(i) is the duration of the initial piece
                step = T(i) * integralFrac;
                // regardless of the spatial and temporal constraints which have already been elamited 
                // through diff, the rest can be represented in the form of integration
                // for each piece, calculate the integrated penalty of the piece using discrete methods
                for (int j=0; j<=integralResolution; j++)
                {   
                    // the jth time
                    s1 = j * step;
                    s2 = s1 * s1;
                    s3 = s2 * s1;
                    s4 = s2 * s2;
                    s5 = s4 * s1;
                    // betai means the ith deraviation of the beta function list at the jth time
                    beta0(0) = 1.0, beta0(1) = s1, beta0(2) = s2, beta0(3) = s3, beta0(4) = s4, beta0(5) = s5;
                    beta1(0) = 0.0, beta1(1) = 1.0, beta1(2) = 2.0 * s1, beta1(3) = 3.0 * s2, beta1(4) = 4.0 * s3, beta1(5) = 5.0 * s4;
                    beta2(0) = 0.0, beta2(1) = 0.0, beta2(2) = 2.0, beta2(3) = 6.0 * s1, beta2(4) = 12.0 * s2, beta2(5) = 20.0 * s3;
                    beta3(0) = 0.0, beta3(1) = 0.0, beta3(2) = 0.0, beta3(3) = 6.0, beta3(4) = 24.0 * s1, beta3(5) = 60.0 * s2;
                    beta4(0) = 0.0, beta4(1) = 0.0, beta4(2) = 0.0, beta4(3) = 0.0, beta4(4) = 24.0, beta4(5) = 120.0 * s1;
                    // then we can transform the trajectory into position, velocity, acceleration, jer and sna
                    // the value at the jth time
                    pos = c.transpose() * beta0;
                    vel = c.transpose() * beta1;
                    acc = c.transpose() * beta2;
                    jer = c.transpose() * beta3;
                    sna = c.transpose() * beta4;
                    // the flatness of the system enables to calculate the thr, quat, and omg.
                    flatMap.forward(vel, acc, jer, 0.0, 0.0, thr, quat, omg);
                    
                    // User defined penalty function
                    // This part remains to be modified for modular scenario
                    // velocity and angular velocity penalty
                    violaVel = vel.squaredNorm() - velSqrMax;   // velocity 
                    violaOmg = omg.squaredNorm() - omgSqrMax;   // omega: angluar velocity
                    // theta penalty (the orientation of the drone)
                    cos_theta = 1.0 - 2.0 * (quat(1) * quat(1) + quat(2) * quat(2));
                    violaTheta = acos(cos_theta) - thetaMax;
                    // the thrust should in a ball
                    violaThrust = (thr - thrustMean) * (thr - thrustMean) - thrustSqrRadi;

                    // init gradients, all set zero
                    gradThr = 0.0;
                    gradQuat.setZero();
                    gradPos.setZero(), gradVel.setZero(), gradOmg.setZero();
                    pena = 0.0;

                    /* the following blocks calculate penalty and gradients 
                       given the violation degree for each segment,
                       calculate the penalty segment,
                       and the gradient of the penalty segment w.r.t. the violation degree
                       */
                    // the point should in the negative half space for every surface
                    L = hIdx(i);            // the index of the Poly for ith piece
                    K = hPolys[L].rows();   // the Poly's h representation
                    for (int k = 0; k < K; k++)
                    {   
                        // each row in hPolys[L] is a surface
                        outerNormal = hPolys[L].block<1, 3>(k, 0);
                        violaPos = outerNormal.dot(pos) + hPolys[L](k, 3);
                        if (smoothedL1(violaPos, smoothFactor, violaPosPena, violaPosPenaD))
                        {
                            gradPos += weightPos * violaPosPenaD * outerNormal; //gradPos have been set to zero
                            pena += weightPos * violaPosPena;
                        }
                    }
                    
                    // gradVel = 2 * weightVel * sum(violaVelPenaD*vel)
                    if (smoothedL1(violaVel, smoothFactor, violaVelPena, violaVelPenaD))
                    {   
                        // there is a grammer trick to save memory
                        // try to think in c++ complier, using the perspective of memory allocationaz
                        gradVel += weightVel * violaVelPenaD * 2.0 * vel;   // the += here is to make full usage of memory
                        pena += weightVel * violaVelPena;
                    }

                    if (smoothedL1(violaOmg, smoothFactor, violaOmgPena, violaOmgPenaD))
                    {
                        gradOmg += weightOmg * violaOmgPenaD * 2.0 * omg;
                        pena += weightOmg * violaOmgPena;
                    }

                    if (smoothedL1(violaTheta, smoothFactor, violaThetaPena, violaThetaPenaD))
                    {
                        gradQuat += weightTheta * violaThetaPenaD /
                                    sqrt(1.0 - cos_theta * cos_theta) * 4.0 *
                                    Eigen::Vector4d(0.0, quat(1), quat(2), 0.0);
                        pena += weightTheta * violaThetaPena;
                    }

                    if (smoothedL1(violaThrust, smoothFactor, violaThrustPena, violaThrustPenaD))
                    {
                        gradThr += weightThrust * violaThrustPenaD * 2.0 * (thr - thrustMean);
                        pena += weightThrust * violaThrustPena;
                    }
                    
                    // the purpose of backward
                    // guess without evaluating the source code:
                    // the system can be represented by (pos, theta) and their s-order deraviations
                    // the gradient to other state parameters of the system should be passed to
                    // (pos, theta) and related deraviations through the chain rule
                    // the totalGrad will then be transmitted to (c, T)
                    // then (c, T) to (xi, T)
                    flatMap.backward(gradPos, gradVel, gradThr, gradQuat, gradOmg,
                                     totalGradPos, totalGradVel, totalGradAcc, totalGradJer,
                                     totalGradPsi, totalGradPsiD);
                    // calculate the totalGrad given the jth grad based on flat transformation

                    // node is the integration discretion coeff
                    node = (j == 0 || j == integralResolution) ? 0.5 : 1.0;
                    alpha = j * integralFrac;

                    // penalty gradient w.r.t. C and T
                    gradC.block<6, 3>(i * 6, 0) += (beta0 * totalGradPos.transpose() +
                                                    beta1 * totalGradVel.transpose() +
                                                    beta2 * totalGradAcc.transpose() +
                                                    beta3 * totalGradJer.transpose()) *
                                                   node * step;
                    gradT(i) += (totalGradPos.dot(vel) +
                                 totalGradVel.dot(acc) +
                                 totalGradAcc.dot(jer) +
                                 totalGradJer.dot(sna)) *
                                    alpha * node * step +
                                node * integralFrac * pena;
                    // integration of total cost 
                    cost += node * step * pena;
                }
            }

            return;
        }

        // key function in this project
        static inline double costFunctional(void *ptr,
                                            const Eigen::VectorXd &x,
                                            Eigen::VectorXd &g)
        {   
            // x is current state
            // g is current gradient
            MODULAR_PolytopeSFC &obj = *(MODULAR_PolytopeSFC *) ptr;
            // tau and xi -- temperal and spatial diffeo
            const int dimTau = obj.temporalDim;
            const int dimXi = obj.spatialDim;
            const double weightT = obj.rho;
            // a class template that allows you to create Eigen expression objects 
            // that map to existing data in memory, without making a copy of that data.
            // This can be useful when you want to use Eigen's functionality on data 
            // that is already stored in a different format or provided by an external library.
            Eigen::Map<const Eigen::VectorXd> tau(x.data(), dimTau);
            Eigen::Map<const Eigen::VectorXd> xi(x.data() + dimTau, dimXi);
            Eigen::Map<Eigen::VectorXd> gradTau(g.data(), dimTau);
            Eigen::Map<Eigen::VectorXd> gradXi(g.data() + dimTau, dimXi);
            
            forwardT(tau, obj.times); // from tau to time period t
            forwardP(xi, obj.vPolyIdx, obj.vPolytopes, obj.points); // from xi to points q

            double cost;
            // minco is a class to calculate the forward and backward transformation
            // between (q, t) and (c, t)
            // as well as related gradients
            // essential preparetions 
            obj.minco.setParameters(obj.points, obj.times); // set A and b 
            obj.minco.getEnergy(cost);
            obj.minco.getEnergyPartialGradByCoeffs(obj.partialGradByCoeffs);
            obj.minco.getEnergyPartialGradByTimes(obj.partialGradByTimes);

            // penalty gradients w.r.t. c and T
            attachPenaltyFunctional(obj.times, obj.minco.getCoeffs(),
                                    obj.hPolyIdx, obj.hPolytopes,
                                    obj.smoothEps, obj.integralRes,
                                    obj.magnitudeBd, obj.penaltyWt, obj.flatmap,
                                    cost, obj.partialGradByTimes, obj.partialGradByCoeffs);

            // penalty gradients w.r.t. p and T
            // reuiqre obj.partialGradByTimes, obj.partialGradByCoeffs calculated from the function above
            // you may use this function without looking into it if strictly designing the attachPenaltyFunctional
            obj.minco.propogateGrad(obj.partialGradByCoeffs, 
                                    obj.partialGradByTimes,
                                    obj.gradByPoints, 
                                    obj.gradByTimes);

            // add time cost to toal const
            cost += weightT * obj.times.sum();
            // obtain an array expression from a vector or matrix.
            // This allows you to perform element-wise operations on the vector.
            // when you perform element-wise operations on the array expression
            // it doesn't modify the original vector `vec`.
            obj.gradByTimes.array() += weightT; // seems useless
            // the auther may want to add temperol penalty gradient w.r.t. T, but seems this will not change
            // the value of gradByTimes, since the array() function will create a new memory and copy

            // penalty gradients w.r.t. xi and tau
            backwardGradT(tau, obj.gradByTimes, gradTau);
            backwardGradP(xi, obj.vPolyIdx, obj.vPolytopes, obj.gradByPoints, gradXi);
            // collision penalty (spacial constraints gradient w.r.t. xi)
            normRetrictionLayer(xi, obj.vPolyIdx, obj.vPolytopes, cost, gradXi);

            return cost;
        }

        // called in getShortestPath
        // used to calculate the cost of a path when initializing
        // given xi, calculate the distance and related gradients
        // counterpart of the costFunctional
        // you do not need to modify this function when implying the modular trajtory planning method
        static inline double costDistance(void *ptr,
                                          const Eigen::VectorXd &xi,
                                          Eigen::VectorXd &gradXi)
        {
            void **dataPtrs = (void **)ptr;
            const double &dEps = *((const double *)(dataPtrs[0]));
            const Eigen::Vector3d &ini = *((const Eigen::Vector3d *)(dataPtrs[1]));
            const Eigen::Vector3d &fin = *((const Eigen::Vector3d *)(dataPtrs[2]));
            const PolyhedraV &vPolys = *((PolyhedraV *)(dataPtrs[3]));

            double cost = 0.0;
            const int overlaps = vPolys.size() / 2;

            Eigen::Matrix3Xd gradP = Eigen::Matrix3Xd::Zero(3, overlaps);
            Eigen::Vector3d a, b, d;
            Eigen::VectorXd r;
            double smoothedDistance;
            for (int i = 0, j = 0, k = 0; i <= overlaps; i++, j += k)
            {
                a = i == 0 ? ini : b;
                if (i < overlaps)
                {
                    k = vPolys[2 * i + 1].cols();
                    Eigen::Map<const Eigen::VectorXd> q(xi.data() + j, k);
                    r = q.normalized().head(k - 1);
                    b = vPolys[2 * i + 1].rightCols(k - 1) * r.cwiseProduct(r) +
                        vPolys[2 * i + 1].col(0);
                }
                else
                {
                    b = fin;
                }

                d = b - a;
                smoothedDistance = sqrt(d.squaredNorm() + dEps);
                cost += smoothedDistance;

                if (i < overlaps)
                {
                    gradP.col(i) += d / smoothedDistance;
                }
                if (i > 0)
                {
                    gradP.col(i - 1) -= d / smoothedDistance;
                }
            }

            Eigen::VectorXd unitQ;
            double sqrNormQ, invNormQ, sqrNormViolation, c, dc;
            for (int i = 0, j = 0, k; i < overlaps; i++, j += k)
            {
                k = vPolys[2 * i + 1].cols();
                Eigen::Map<const Eigen::VectorXd> q(xi.data() + j, k);
                Eigen::Map<Eigen::VectorXd> gradQ(gradXi.data() + j, k);
                sqrNormQ = q.squaredNorm();
                invNormQ = 1.0 / sqrt(sqrNormQ);
                unitQ = q * invNormQ;
                gradQ.head(k - 1) = (vPolys[2 * i + 1].rightCols(k - 1).transpose() * gradP.col(i)).array() *
                                    unitQ.head(k - 1).array() * 2.0;
                gradQ(k - 1) = 0.0;
                gradQ = (gradQ - unitQ * unitQ.dot(gradQ)).eval() * invNormQ;

                sqrNormViolation = sqrNormQ - 1.0;
                if (sqrNormViolation > 0.0)
                {
                    c = sqrNormViolation * sqrNormViolation;
                    dc = 3.0 * c;
                    c *= sqrNormViolation;
                    cost += c;
                    gradQ += dc * 2.0 * q;
                }
            }

            return cost;
        }

        // called in setup, used to initialize a feasible solution
        // init a zig-zag path with the optimal length defined in this function
        // you do not need to modify this function when implying the modular trajtory planning method
        static inline void getShortestPath(const Eigen::Vector3d &ini,
                                           const Eigen::Vector3d &fin,
                                           const PolyhedraV &vPolys,
                                           const double &smoothD,
                                           Eigen::Matrix3Xd &path)
        {   
            // vPolys consist of convex hull and their union regeions
            const int overlaps = vPolys.size() / 2;
            Eigen::VectorXi vSizes(overlaps);
            for (int i = 0; i < overlaps; i++)
            {
                vSizes(i) = vPolys[2 * i + 1].cols();
            }
            // the dim of xi to a vPolytope is the number of the vertices
            // init xi through averaging 
            Eigen::VectorXd xi(vSizes.sum());
            for (int i = 0, j = 0; i < overlaps; i++)
            {
                xi.segment(j, vSizes(i)).setConstant(sqrt(1.0 / vSizes(i)));
                j += vSizes(i);
            }

            double minDistance;
            // learn from the grammer of the code lines
            void *dataPtrs[4];
            dataPtrs[0] = (void *)(&smoothD);
            dataPtrs[1] = (void *)(&ini);
            dataPtrs[2] = (void *)(&fin);
            dataPtrs[3] = (void *)(&vPolys);
            lbfgs::lbfgs_parameter_t shortest_path_params;
            shortest_path_params.past = 3;
            shortest_path_params.delta = 1.0e-3;
            shortest_path_params.g_epsilon = 1.0e-5;
            
            // minimize cost distance through lbfgs
            // the target parameters is xi
            lbfgs::lbfgs_optimize(xi,
                                  minDistance,
                                  &MODULAR_PolytopeSFC::costDistance,
                                  nullptr,
                                  nullptr,
                                  dataPtrs,     // dataPtr and this
                                  shortest_path_params);

            // the inner path point should exist in the join of two polyhedro
            path.resize(3, overlaps + 2);
            path.leftCols<1>() = ini;   // pre-defined by user
            path.rightCols<1>() = fin;  // pre-defined by user
            Eigen::VectorXd r;
            for (int i = 0, j = 0, k; i < overlaps; i++, j += k)
            {
                k = vPolys[2 * i + 1].cols();
                // x.data()+j: a pointer to the data array `xi` offset by `j` elements.
                Eigen::Map<const Eigen::VectorXd> q(xi.data() + j, k);
                r = q.normalized().head(k - 1);
                // path is the points list, expressed in the world coordinate
                path.col(i + 1) = vPolys[2 * i + 1].rightCols(k - 1) * r.cwiseProduct(r) +
                                  vPolys[2 * i + 1].col(0);
            }

            return;
        }

        // called in setup, to ensure the feasiblity of a safeCorridor
        // and convert it to the v-representation (with the joins of the polytopes)
        static inline bool processCorridor(const PolyhedraH &hPs,
                                           PolyhedraV &vPs)
        {   
            // vPs is the list convexhull and thier joints
            const int sizeCorridor = hPs.size() - 1;

            vPs.clear();
            vPs.reserve(2 * sizeCorridor + 1);

            int nv;
            PolyhedronH curIH;
            PolyhedronV curIV, curIOB;
            for (int i = 0; i < sizeCorridor; i++)
            {   
                // check the correctness of the hPolytope
                // if right, return related v presentation
                if (!geo_utils::enumerateVs(hPs[i], curIV))
                {
                    return false;
                }
                nv = curIV.cols();
                curIOB.resize(3, nv);
                curIOB.col(0) = curIV.col(0);
                curIOB.rightCols(nv - 1) = curIV.rightCols(nv - 1).colwise() - curIV.col(0);
                vPs.push_back(curIOB);

                curIH.resize(hPs[i].rows() + hPs[i + 1].rows(), 4);
                curIH.topRows(hPs[i].rows()) = hPs[i];
                curIH.bottomRows(hPs[i + 1].rows()) = hPs[i + 1];
                if (!geo_utils::enumerateVs(curIH, curIV))
                {
                    return false;
                }
                nv = curIV.cols();
                curIOB.resize(3, nv);
                curIOB.col(0) = curIV.col(0);
                curIOB.rightCols(nv - 1) = curIV.rightCols(nv - 1).colwise() - curIV.col(0);
                vPs.push_back(curIOB);
            }

            if (!geo_utils::enumerateVs(hPs.back(), curIV))
            {
                return false;
            }
            nv = curIV.cols();
            curIOB.resize(3, nv);
            curIOB.col(0) = curIV.col(0);
            curIOB.rightCols(nv - 1) = curIV.rightCols(nv - 1).colwise() - curIV.col(0);
            vPs.push_back(curIOB);

            return true;
        }

        // called in the optimize function
        static inline void setInitial(const Eigen::Matrix3Xd &path,
                                      const double &speed,
                                      const Eigen::VectorXi &intervalNs,
                                      Eigen::Matrix3Xd &innerPoints,
                                      Eigen::VectorXd &timeAlloc)
        {   
            // path is the initial path generated through RRT ?
            // how do we inite intervalNs ?

            // intervalNs is like [2, 4, 5, 3]
            const int sizeM = intervalNs.size();
            const int sizeN = intervalNs.sum();
            // change the size of an Eigen object without losing the existing data 
            // if the new size is larger than the current size. If the new size is smaller, 
            // the excess data is discarded.
            innerPoints.resize(3, sizeN-1);
            timeAlloc.resize(sizeN);

            Eigen::Vector3d a, b, c;
            for (int i = 0, j = 0, k = 0, l; i < sizeM; i++)
            {   
                // the first interval
                l = intervalNs(i);
                // the start point of the interval
                a = path.col(i);
                // the end point of the interval
                b = path.col(i+1);
                // c is the average segment of vector a->b
                c = (b-a)/l;
                //! @todo this part may be important for the following module trajectory adjustment
                // the time duration is set according to the speed and distance.
                // for a specific piece in path generated by RRT
                // assign l segments to it
                // and each segments is attached to a duration based on its norm and speed
                // this may used to initialzed the time T  
                timeAlloc.segment(j, l).setConstant(c.norm()/speed);
                j += l;

                for (int m=0; m<l; m++)
                {
                    if (i>0 || m>0)
                    {
                        // for each piece
                        // assign the segment points
                        innerPoints.col(k++) = a + c*m;
                    }
                }
            }

        }

    public:
        // call setup first before run optimize
        // magnitudeBounds = [v_max, omg_max, theta_max, thrust_min, thrust_max]^T
        // penaltyWeights = [pos_weight, vel_weight, omg_weight, theta_weight, thrust_weight]^T
        // physicalParams = [vehicle_mass, gravitational_acceleration, horitonral_drag_coeff,
        //                   vertical_drag_coeff, parasitic_drag_coeff, speed_smooth_factor]^T
        inline bool setup(const double &timeWeight,
                          const Eigen::Matrix3d &initialPVA,
                          const Eigen::Matrix3d &terminalPVA,
                          const PolyhedraH &safeCorridor,
                          const double &lengthPerPiece,
                          const double &smoothingFactor,
                          const int &integralResolution,
                          const Eigen::VectorXd &magnitudeBounds,
                          const Eigen::VectorXd &penaltyWeights,
                          const Eigen::VectorXd &physicalParams)
        {
            rho = timeWeight;
            headPVA = initialPVA;
            tailPVA = terminalPVA;
            // hPolytopes is safeCorridor
            hPolytopes = safeCorridor;
            for (size_t i = 0; i < hPolytopes.size(); i++)
            {   
                // extract the first 3 left cols from a hPolytope
                // hPolytopes: [h0,h1,h2,h3; ....  ]
                // for each hPolytop, h0*x + h1*y + h2*z + h3 = 0   
                const Eigen::ArrayXd norms =
                    hPolytopes[i].leftCols<3>().rowwise().norm();
                // rowwise() to perform operations on a row-wise basis rather than 
                // applying operations element-wise or column-wise.
                
                // hPoly normalization
                hPolytopes[i].array().colwise() /= norms;
            }

            // check corridor and covert it to the v-representations
            if (!processCorridor(hPolytopes, vPolytopes))
            {
                return false;
            }

            polyN = hPolytopes.size();
            smoothEps = smoothingFactor;
            integralRes = integralResolution;
            magnitudeBd = magnitudeBounds;
            penaltyWt = penaltyWeights;
            physicalPm = physicalParams;
            allocSpeed = magnitudeBd(0) * 3.0;

            // init a shallow path
            // the path consists of 2*N-1 points
            // the path is splitted into multiple pieces according to the lengthPerPiece parameter
            // vPolytopes is of 2*polyN-1
            getShortestPath(headPVA.col(0), tailPVA.col(0),
                            vPolytopes, smoothEps, shortPath);
            const Eigen::Matrix3Xd deltas = shortPath.rightCols(polyN) - shortPath.leftCols(polyN);
            // the piece number in one hPolytope
            pieceIdx = (deltas.colwise().norm() / lengthPerPiece).cast<int>().transpose();
            pieceIdx.array() += 1;
            // the total piece number
            pieceN = pieceIdx.sum();
            
            // setup the polytope representation, do not need to be modified
            temporalDim = pieceN;
            spatialDim = 0;

            vPolyIdx.resize(pieceN - 1);
            hPolyIdx.resize(pieceN);
            for (int i = 0, j = 0, k; i < polyN; i++)
            {
                k = pieceIdx(i);
                for (int l = 0; l < k; l++, j++)
                {
                    if (l < k - 1)
                    {   
                        // the jth piece is in the 2*ith vPoly of the vPolySequence
                        vPolyIdx(j) = 2 * i;
                        spatialDim += vPolytopes[2 * i].cols();
                    }
                    else if (i < polyN - 1) // k is the last piece in the hPolytopes
                    {   
                        // the last piece in the hPolytopes locates in the joints of two
                        vPolyIdx(j) = 2 * i + 1;
                        spatialDim += vPolytopes[2 * i + 1].cols();
                    }
                    // the jth piece is in the ith hPoly of the hPolySequence
                    hPolyIdx(j) = i;
                }
            }

            // Setup for MINCO_S3NU, FlatnessMap, and L-BFGS solver
            minco.setConditions(headPVA, tailPVA, pieceN);
            // head pos, vel, and acc; tial pos, vel, and acc; pieces number
            //! @todo modify this later to include the crane-module system dynamics
            flatmap.reset(physicalPm(0), physicalPm(1), physicalPm(2),
                          physicalPm(3), physicalPm(4), physicalPm(5));
            // physical parameters to forward and backward the system dynamics

            // Allocate temporate variables
            points.resize(3, pieceN - 1);
            times.resize(pieceN);
            gradByPoints.resize(3, pieceN - 1);
            gradByTimes.resize(pieceN);
            partialGradByCoeffs.resize(6 * pieceN, 3);
            partialGradByTimes.resize(pieceN);

            return true;
        }

        inline double optimize(Trajectory<5> &traj,
                               const double &relCostTol)
        {
            Eigen::VectorXd x(temporalDim + spatialDim);
            Eigen::Map<Eigen::VectorXd> tau(x.data(), temporalDim);
            Eigen::Map<Eigen::VectorXd> xi(x.data() + temporalDim, spatialDim);

            setInitial(shortPath, allocSpeed, pieceIdx, points, times);
            backwardT(times, tau);
            backwardP(points, vPolyIdx, vPolytopes, xi);

            // L-BFGS optimization
            double minCostFunctional;
            lbfgs_params.mem_size = 256;
            lbfgs_params.past = 3;
            lbfgs_params.min_step = 1.0e-32;
            lbfgs_params.g_epsilon = 0.0;
            lbfgs_params.delta = relCostTol;
            // Eigen::VectorXd, double
            // funcHandler, proc_stepbound funcHandler, proc_progress funcHandler
            // the GCOPTER_PolytopeSFC class instance
            // lbfgs_params
            int ret = lbfgs::lbfgs_optimize(x,
                                            minCostFunctional,                      // cost
                                            &MODULAR_PolytopeSFC::costFunctional,   // function to be optimized
                                            nullptr,                                // not important
                                            nullptr,                                // not important
                                            this,                                   // instance to provide callback data
                                            lbfgs_params);
            
            // fx = costFunctional(this instance, x, g);
            // costFunctional should provide some API
            // costFunctional (this, x, g);
            // x: current state; g: gradient at current state;
            // give x, calculate g;
            // ret is the optimization result
            if (ret >= 0)
            {   
                // if success, get times and points from optimized tau and xi
                forwardT(tau, times);
                forwardP(xi, vPolyIdx, vPolytopes, points);
                // points: trajectory pieces' end;
                // times : trajectory pieces' duration;

                // Then you can get the MINCO representation according to the points and times
                // using the following code
                minco.setParameters(points, times);
                // the trajectory can be loaded after setParameters
                minco.getTrajectory(traj);
            }
            else
            {
                // if fail, return ERROR
                traj.clear();
                minCostFunctional = INFINITY;
                std::cout << "Optimization Failed: "
                          << lbfgs::lbfgs_strerror(ret)
                          << std::endl;
            }

            return minCostFunctional;
        }
    };
}

#endif