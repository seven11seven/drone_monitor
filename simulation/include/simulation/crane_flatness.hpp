#ifndef CRANE_FLATNESS_HPP
#define CRANE_FLATNESS_HPP

#include <Eigen/Eigen>
#include <cmath>

namespace crane_flatness
{   
    // regard the crane-module as simple linear system
    // the pos, vec, and acc of module can uniquely define the parameters of the crane-module system
    // e.g. the flat output is pos, vec, and acc
    class SimpleFlatnessMap
    {
    public:
        inline void reset(const double &module_mass,
                          const double &jib_mass,
                          const double &trolley_mass,
                          const double &jib_innt,
                          const double &trolley_innt,
                          const double &gravitational_acceleration,
                          const double &x,
                          const double &y,
                          const double &h,
                          const double &l)
        {   
            // used in modular.hpp to setup parameters
            m_mass = module_mass;
            j_mass = jib_mass;
            t_mass = trolley_mass;
            j_innt = jib_innt;
            t_innt = trolley_innt;
            grav = gravitational_acceleration;  // +9.8 as default
            // the coordinates of the jib root; the mass and inertia of the jib and trolley
            tc_x = x;
            tc_y = y;
            tc_h = h;
            L = l;
            return;
        }
        
        inline void forward(const Eigen::Vector3d &pos,
                            const Eigen::Vector3d &vel,
                            const Eigen::Vector3d &acc,
                            double &jib_omg,      // jib angular velocity
                            double &trolley_vel,  // trolley velocity
                            double &jib_force,
                            double &trolley_force,
                            double &module_force)
        {   
            // store neccessary values that will be used in backward
            // Assumptions:
            // 1. No rotation during hoisting
            // 2. The rope is vertical to the ground
            // 3. rigidly attached to the trolley
            double sint, cost;
            double t_acc0;
            double j_acc0, j_acc1, j_acc2, j_acc3;
            
            // v0, v1, v2, a0, a1, a2
            v0 = vel(0);
            v1 = vel(1);
            v2 = vel(2);
            a0 = acc(0);
            a1 = acc(1);
            a2 = acc(2);
            p0 = pos(0) - tc_x;
            p1 = pos(1) - tc_y;
            p2 = pos(2) - tc_h;
            // j_pos, j_vel, t_pos, t_vel
            sq_t_pos = p0*p0 + p1*p1;
            t_pos = sqrt(sq_t_pos);
            sint = p1 / t_pos;
            cost = p0 / t_pos;
            arcTheta(cost, sint, j_pos);
            t_vel = (v0*p0 + v1*p1) / t_pos;
            j_vel = (p0*p0*v0 + p0*p1*v1) / (sq_t_pos*p1) - v0/p1;
            // j_acc, t_acc
            t_acc0 = a0*p0 + v0*v0*p0 + v1*v1*p1 + p1*a1;
            t_acc = t_acc0/t_pos - t_vel*t_vel/t_pos;
            j_acc0 = 2*p0*v0*v0 + p0*p0*a0 + v0*v1*p1 + p0*v1*v1 + p0*p1*a1;
            j_acc1 = p0*p0*v0 + p0*p1*v1;
            j_acc2 = (2*p0*v0 + 2*p1*v1) * v1 + sq_t_pos * v1;
            j_acc3 = sq_t_pos * p1;
            j_acc  = j_acc0/j_acc3 - j_acc1*j_acc2/(j_acc3*j_acc3) - a0/p1 + v0*v1/(p1*p1);
            // mf0, mf1, mf2
            mf0 = m_mass * a0;
            mf1 = m_mass * a1;
            mf2 = m_mass * (a2+grav);
            // j_force, t_force, m_force
            m_force = sqrt(mf0*mf0 + mf1*mf1 + mf2*mf2);
            t_force = (t_mass+m_mass)*(t_acc - t_pos*j_vel*j_vel);
            j_force = (t_mass+m_mass)*(2*t_pos*t_vel*j_vel + t_pos*t_pos*j_acc) + \
                      2 * (j_innt + t_innt + j_mass*L*L) * j_vel * j_acc;
            
            // parameters with constraints
            jib_omg = j_vel;      
            trolley_vel = t_vel;
            jib_force = j_force;
            trolley_force = t_force;
            module_force = m_force;
        }

        inline void backward(const Eigen::Vector3d &pos_grad,
                             const Eigen::Vector3d &vel_grad,
                             const Eigen::Vector3d &acc_grad,
                             const double &omg_grad,
                             Eigen::Vector3d &pos_total_grad,
                             Eigen::Vector3d &vel_total_grad,
                             Eigen::Vector3d &acc_total_grad,
                             Eigen::Vector3d &jer_total_grad)
        {
            // pos_grad, vel_grad: gradient passed from violaPos and violaVel
            // omg_grad, mfr_grad: gradient passed from violaOmg and violaForce
            double omg_p0, omg_p0_0, omg_p0_1;
            double tempa, tempb;
            omg_p0_0 = 2*p0*v0 + p1*v1;
            omg_p0_1 = 2*p0*p0*p0*p1*v0 + 2*p0*p0*p1*p1*v1;
            tempa = sq_t_pos*p1;
            tempb = tempa*tempa;
            omg_p0 = omg_p0_0/tempa - omg_p0_1/tempb;
            double omg_v0;
            omg_v0 = p0*p0/tempa - 1/p1;
            double omg_p1, omg_p1_0;
            omg_p1_0 = p0*p1*v1*(p0*p0+3*p1*p1);
            omg_p1 = p0*v1/tempa - omg_p1_0/tempb + v0/(p1*p1);
            double omg_v1;
            omg_v1 = p0*p1/tempa;

            pos_total_grad(0) = omg_grad * omg_p0 + pos_grad(0);
            pos_total_grad(1) = omg_grad * omg_p1 + pos_grad(1);
            pos_total_grad(2) = pos_grad(2);
            vel_total_grad(0) = omg_grad * omg_v0 + vel_grad(0);
            vel_total_grad(1) = omg_grad * omg_v1 + vel_grad(1);
            vel_total_grad(2) = vel_grad(2);
            acc_total_grad(0) = acc_grad(0);
            acc_total_grad(1) = acc_grad(1);
            acc_total_grad(2) = acc_grad(2);
            jer_total_grad(0) = 0;
            jer_total_grad(1) = 0;
            jer_total_grad(2) = 0;
        }

        inline void arcTheta(const double c,
                             const double s,
                             double &t)
        {
            t = asin(s);
            if(c < 0) t = M_PI-t;
        }

    private:
        // setup parameters
        double L;                // the loctaion of the jib's CoM w.r.t. the rotation point 
        double m_mass, j_mass, t_mass;
        double j_innt, t_innt;
        double grav;
        double tc_x, tc_y, tc_h;    // coordinate of the rotation point 
        double p0, p1, p2;          // vector from rotation point to the module CoG
        // joint parameters
        double v0, v1, v2, a0, a1, a2;
        double j_pos, j_vel, t_pos, t_vel, sq_t_pos;
        double j_acc, t_acc;
        // force
        double j_force, t_force, m_force;
        double mf0, mf1, mf2;

    };

    // class RotationFlatnessMap
}

#endif