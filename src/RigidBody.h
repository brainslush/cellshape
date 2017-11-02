/*
 * Handles rigid body simulation
 */

#pragma once

#include <iostream>
#include <set>
#include <eigen3/Eigen/Eigen>
#include "Eigen.h"

#ifndef SRC_RIGIDBODY_H_
#define SRC_RIGIDBODY_H_

namespace physic {
    using Vec3 = Eigen::Vector3d;

    /*
     * function to sum up quaternions
     */

    template<typename L, typename R>
    auto qsum(L l, R r) {
        Eigen::Quaterniond c;
        c.coeffs() = l.coeffs() + r.coeffs();
        return c;
    };

    /*
     * function to get the difference between two quaternions
     */

    template<typename L, typename R>
    auto qdiff(L l, R r) {
        Eigen::Quaterniond c;
        c.coeffs() = l.coeffs() - r.coeffs();
        return c;
    };

    /*
     * function to convert a rotation vector to quaternion
     */
    Eigen::Quaterniond vec2quat(const Vec3 &v);

    /*
     * function to calculate the angle between two vectors
     */

    double angleVector2d(const Vec3 &v);

    double angleVector2d(
            const double &x1,
            const double &y1,
            const double &x2,
            const double &y2);

    /*
     * Functor for torque and force calculation
     * */

    class RigidBody3d;

    class functor {
    public:

        functor() = default;

        virtual ~functor() = default;

        virtual std::pair<Vec3, Vec3> calc(
                const Vec3 &X,
                const Vec3 &v,
                //const Eigen::Quaterniond &R,
                const double &R,
                const Vec3 &w,
                RigidBody3d &rigidBody
        ) {
            return {Vec3(0, 0, 0), Vec3(0, 0, 0)};
        };

        virtual Vec3 calc_force(
                const Vec3 &X,
                const Vec3 &v,
                //const Eigen::Quaterniond &R,
                const double &R,
                const Vec3 &w,
                RigidBody3d &rigidBody
        ) {
            return Vec3(0, 0, 0);
        };

        virtual Vec3 calc_torque(
                const Vec3 &X,
                const Vec3 &v,
                //Eigen::Quaterniond &R,
                const double &R,
                const Vec3 &w,
                RigidBody3d &rigidBody
        ) {
            return Vec3(0, 0, 0);
        };
    };

    /*
     * Rigid body class which uses numerical integrators to calculate changes
     * */

    class Base;

    class RigidBody3d {
    public:
        explicit RigidBody3d(Base *obj) :
                object(obj),
                functors(nullptr) {
        };

        RigidBody3d(
                Base *obj,
                Vec3 iX,
                //Eigen::Quaterniond iQ, // rotation in lab frame
                double iR,
                Eigen::Matrix3d iI,
                double iM,
                //double iEpsilon,
                std::set<functor *> *iFunctors
        ) :
                object(obj),
                X(std::move(iX)),
                //q(iQ),
                R(iR),
                M(iM),
                //epsilon(iEpsilon),
                functors(iFunctors) {
            I = iI.diagonal();
            v = Vec3(0, 0, 0);
            w = Vec3(0, 0, 0);
            F = Vec3(0, 0, 0);
            T = Vec3(0, 0, 0);
        };

        RigidBody3d(
                Base *obj,
                Vec3 iX,
                Vec3 iV,
                //Eigen::Quaterniond iQ, // rotation in lab frame
                double iR,
                Vec3 iw,
                Eigen::Matrix3d iI,
                double iM,
                std::set<functor *> *iFunctors
        ) :
                object(obj),
                X(std::move(iX)),
                v(std::move(iV)),
                //q(iQ),
                R(iR),
                w(std::move(iw)),
                I(iI.diagonal()),
                M(iM),
                //epsilon(iEpsilon),
                functors(iFunctors) {
            F = Eigen::Vector3d(0, 0, 0);
            T = Eigen::Vector3d(0, 0, 0);
        };

        virtual ~RigidBody3d() = default;

        virtual Vec3 &get_X() {
            return X;
        };

        virtual double &get_R() {
            //return q.toRotationMatrix();
            return R;
        };

        /*
        virtual Eigen::Quaterniond &get_q() {
            return q;
        };
         */

        virtual Vec3 &get_v() {
            return v;
        };

        virtual Vec3 &get_w() {
            return w;
        };

        virtual double &get_M() {
            return M;
        };

        virtual Vec3 &get_I() {
            return I;
        };

        virtual Vec3 &get_F() {
            return F;
        }

        virtual Vec3 &get_T() {
            return T;
        }

        Base *get_object() {
            return object;
        };

        virtual void set_object(Base *iObj) {
            object = iObj;
        };

        virtual void set_position(const Vec3 &iX) {
            X = iX;
        }

        virtual void set_rotation(const double &iR) {
            R = iR;
        }

        virtual void set_inertia(const Eigen::Matrix3d &iI) {
            I = iI.diagonal();
        };

        virtual void set_mass(const double &iM) {
            M = iM;
        };

        virtual void set_velocity(const Vec3 &iV) {
            v = iV;
        }

        /*
        virtual void add_force(const Vec3 &iX, const Vec3 &iF) {
            F += iF;
            T += (iX - X).cross(iF);
        };
         */

        /*
         * Simulate single time step
         * */

        void do_timeStep(double &dT) {
            // update force and torque
            auto _temp = sum_functors(X, v, R, w);
            F = _temp.first;
            T = _temp.second;
            // do simulation
            do_rk4(dT);
            // ceck that R doesn't exceed 2Pi
            R = fmod(R, 2.0 * M_PI);
        }

    protected:
        Vec3 X; // position
        //Eigen::Quaterniond q; // quaternion
        double R;
        Vec3 I; // moment of inertia diagonalized.
        double M; // mass
        //double epsilon; // precision for rotation calculation
        std::set<functor *> *functors; // functors which calculate forces and torques
        Vec3 v; // velocity
        Vec3 w; // angular momentum
        Vec3 F; // current force
        Vec3 T; // current torque
        Base *object;

        std::pair<Vec3, Vec3> sum_functors(
                const Vec3 &iX,
                const Vec3 &iv,
                //const Eigen::Quaterniond &iR,
                const double &iR,
                const Vec3 &iL
        ) {
            Vec3 _f(0, 0, 0);
            Vec3 _t(0, 0, 0);
            if (functors) {
                for (auto &_it : *functors) {
                    auto _c = _it->calc(iX, iv, iR, iL, *this);
                    _f = _f + _c.first;
                    _t = _t + _c.second;
                }
            }
            return {_f, _t};
        }

        /*
         * sum up force functors
         */

        auto calc_force(
                const Vec3 &iX,
                const Vec3 &iv,
                //Eigen::Quaterniond &iR,
                const double &iR,
                const Vec3 &iL
        ) {
            Vec3 ret;
            if (functors) {
                for (auto &it : *functors) {
                    ret += it->calc_force(iX, iv, iR, iL, *this);
                }
            }
            return ret;
        }

        /*
         * sum up torque functors
         * */

        Vec3 calc_torque(
                const Vec3 &iX,
                const Vec3 &iv,
                //Eigen::Quaterniond &iR,
                const double &iR,
                const Vec3 &iL
        ) {
            Vec3 ret;
            if (functors) {
                for (auto &it : *functors) {
                    ret += it->calc_torque(iX, iv, iR, iL, *this);
                }
            }
            return ret;
        }

        void do_leapFrog(double &dT) {
            Vec3 _a = F / M;
            Vec3 _X = X + dT * v + 0.5 * dT * dT * _a;
            //Vec3 _v =
        }

        /*
         * Numerical integration method based on the 4th order Runge-Kutta algorithm
         * */

        void do_rk4(double &dT) {

            // make sure q is normalized
            //q.normalize();
            // get quaternion rotation as rotation matrix form
            //auto _R = q.toRotationMatrix();
            // calculate inverse of
            //auto _Ib = _R * I;
            Vec3 _Ii = I.cwiseInverse();

            // angular momentum in body frame
            //auto _wb1 = _R * w;

            // calculate first half time step
            Vec3 _a1 = F / M;
            Vec3 _v2 = v + 0.5 * dT * _a1;
            Vec3 _x2 = X + 0.5 * dT * _v2;
            Vec3 _t1 = _Ii.cwiseProduct(T);
            Vec3 _w2 = w + 0.5 * dT * _t1;
            //auto _wb2 = _R * _w2;
            //auto _q2 = qsum(q, qscale(0.25 * dT, q * vec2quat(_wb2)));
            auto _R2 = R + 0.5 * dT * _w2(2);

            auto _k2 = sum_functors(_x2, _v2, _R2, _w2);

            // calculate another half time step with new values
            Vec3 _a2 = _k2.first / M;
            Vec3 _v3 = v + 0.5 * dT * _a2;
            Vec3 _x3 = X + 0.5 * dT * _v3;
            Vec3 _t2 = _Ii.cwiseProduct(_k2.second);
            Vec3 _w3 = w + 0.5 * dT * _t2;
            //auto _wb3 = _R * _w3;
            //auto _q3 = qsum(q, qscale(0.25 * dT, q * vec2quat(_wb3)));
            auto _R3 = R + 0.5 * dT * _w3(2);

            auto _k3 = sum_functors(_x3, _v3, _R3, _w3);

            // and do it a third time with a full time step
            Vec3 _a3 = _k3.first / M;
            Vec3 _v4 = v + dT * _a3;
            Vec3 _x4 = X + dT * _v4;
            Vec3 _t3 = _Ii.cwiseProduct(_k3.second);
            Vec3 _w4 = w + dT * _t3;
            //auto _wb4 = _R * _w4;
            //auto _q4 = qsum(q, qscale(0.5 * dT, q * vec2quat(_wb4)));
            auto _R4 = R + dT * _w4(2);

            auto _k4 = sum_functors(_x4, _v4, _R4, _w4);

            // obtain new position and velocity, roation, angular momemntum
            X = X + dT / 6.0 * (v + 2 * _v2 + 2 * _v3 + _v4);
            v = v + dT / 6.0 * (_a1 + 2 * _a2 + 2 * _a3 + _k4.first / M);
            //q = qsum(q, qscale(dT / 12, q * vec2quat(_wb1 + 2 * _wb2 + 2 * _wb3 + _wb4)));
            R = R + dT / 6.0 * (w(2) + 2 * _w2(2) + 2 * _w3(2) + _w4(2));
            w = w + dT / 6.0 * (_t1 + 2 * _t2 + 2 * _t3 + _Ii.cwiseProduct(_k4.second));

        }

        /*
         * Numerical integration method based on a modified version of
         * the velocity verlet algorithm
         * */

        /*
        void do_vverlet(double &dT) {
            auto _a = F / M;
            // translation part 1
            X = X + dT * v + 0.5 * dT * dT * _a; // calculating new position
            // rotational part 1
            q.normalize();
            // transforming quaterinon to rotation matrix increases speed
            auto _R = q.toRotationMatrix();
            std::cout << "This is R:\n" << _R << "\n";
            std::cout << "This is L:\n" << w << "\n";
            // angular momentum in body frame
            auto _Lb = _R * w;
            std::cout << "This is L in body frame:  \n" << _Lb << "\n";
            std::cout << "This is T:\n" << _Lb << "\n";
            // torque in body frame
            auto _Tb = _R * T;
            std::cout << "This is T in body frame:  \n" << _Tb << "\n";
            // moment of inertia in body frame diagonalized and inversed
            auto _Ib = _R * I;
            _Ib = _Ib.cwiseInverse();
            std::cout << "This is I⁻¹m:\n" << _Ib << "\n";
            // angular momentum in body frame after half time step
            auto _Lbt2 = _Lb + 0.5 * dT * (_Tb - (_Ib.cwiseProduct(_Lb)).cross(_Lb));
            // quaternion at half time step at iteration k = 0
            auto _qkt2 = qsum(q, q * vec2quat(_Ib.cwiseProduct(0.25 * dT * _Lbt2)));
            // angular momentum in lab frame
            Eigen::Quaterniond _Lwt2;
            _Lwt2.w() = 0;
            _Lwt2.vec() = w + 0.5 * dT * T;
            // precision itterator
            auto _qk1t2 = _qkt2;
            Eigen::Quaterniond _qdk1t2;
            Vec3 _Lbk1t2;
            do {
                _qkt2 = _qk1t2;
                _Lbk1t2 = (_qkt2 * _Lwt2 * _qkt2.inverse()).vec();
                _qdk1t2 = qscale(0.5, _qkt2) * vec2quat(_Ib.cwiseProduct(_Lbk1t2));
                _qk1t2 = qsum(q, qscale(0.5 * dT, _qdk1t2));
            } while ((qdiff(_qk1t2, _qkt2)).norm() > epsilon);
            q = qsum(q, qscale(dT, _qdk1t2));
            // part 2 estimate L and v w/ an estimated T and a at t+dt
            auto _Ldt = w + dT * T;
            auto _vdt = v + dT * _a;

            _vdt = v + 0.5 * dT * (_a + calc_force(X, _vdt, q, _Ldt)); // estimate vdt
            _Ldt = w + 0.5 * dT * (T + calc_torque(X, _vdt, q, _Ldt)); // estimate ldt
            auto _newFT = sum_functors(X, _vdt, q, _Ldt);
            v = v + 0.5 * dT * (_a + _newFT.first);
            w = w + 0.5 * dT * (T + _newFT.second);
        }
         */
    };


    /*
     * Base class for all physcial objects
     */

    class Base {
    public:
        Base() :
                rigidBody(nullptr) {};

        /*
        Base(RigidBody3d *iRigidBody):
                rigidBody(iRigidBody){}
        */

        virtual ~Base() = default;

        virtual physic::RigidBody3d &get_solver() { return *solver; };

    protected:
        RigidBody3d *solver;
        std::set<physic::functor *> &iFunctors;
    };

};

#endif /* SRC_RIGIDBODY_H_ */
