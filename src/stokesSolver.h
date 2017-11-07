#ifndef CELLFORMATION_STOKESSOLVER_H
#define CELLFORMATION_STOKESSOLVER_H

/*
 * TODO:
 * - add rotation to the simulation.
 */


/*
 * Handles stokes simmulaton
 */

#pragma once

#include <iostream>
#include <set>
#include <eigen3/Eigen/Eigen>
#include "Eigen.h"

namespace stokes {
    using Vec3 = Eigen::Vector3d;

    class Solver;

    /*
     * functor class from which all force/torque functors inherit so that
     * the solver can access them via pointers
     */

    class functor {
    public:

        functor() = default;

        virtual ~functor() = default;

        virtual std::pair<Vec3, Vec3> calc(
                const Vec3 &X,
                const Vec3 &v,
                const double &R,
                const Vec3 &w,
                Solver &solver
        ) {
            return {Vec3(0, 0, 0), Vec3(0, 0, 0)};
        };
    };

    /*
     * Rigid body class which uses numerical integrators to calculate changes
     *
     * Variables:
     * X - position of COM
     * R - rotation measured in rads
     * c - constant of the stokes friction
     * v - velocity of the COM
     * w - angular velocity of the COM
     * F - current force acting on the object
     * T - current torque acting on the object
     *
     */

    class Base;

    class Solver {
    public:

        /*
         * constructors
         */

        explicit Solver(Base *iObj) :
                object(iObj),
                functors(nullptr) {
            X = Vec3(0, 0, 0);
            R = 0.0;
            c = 1.0;
            v = Vec3(0, 0, 0);
            w = Vec3(0, 0, 0);
            F = Vec3(0, 0, 0);
            T = Vec3(0, 0, 0);
        };

        Solver(
                Base *iObj,
                std::set<functor *> *iFunctors
        ) :
                object(iObj),
                functors(iFunctors)
        {
            X = Vec3(0, 0, 0);
            R = 0.0;
            c = 1.0;
            v = Vec3(0, 0, 0);
            w = Vec3(0, 0, 0);
            F = Vec3(0, 0, 0);
            T = Vec3(0, 0, 0);
        }

        Solver(
                Base *obj,
                Vec3 iX,
                double iR,
                double iC,
                std::set<functor *> *iFunctors
        ) :
                object(obj),
                X(std::move(iX)),
                R(iR),
                c(iC),
                functors(iFunctors) {
            v = Vec3(0, 0, 0);
            w = Vec3(0, 0, 0);
            F = Vec3(0, 0, 0);
            T = Vec3(0, 0, 0);
        };

        /*
         * destructor
         */

        virtual ~Solver() = default;

        /*
         * getters
         */


        virtual Vec3 &get_X() {
            return X;
        };

        virtual double &get_R() {
            return R;
        };

        virtual Vec3 &get_v() {
            return v;
        };

        virtual Vec3 &get_w() {
            return w;
        };

        virtual double &get_C() {
            return c;
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

        /*
         * setters
         */

        virtual void set_object(Base *iObj) {
            object = iObj;
        };

        virtual void set_X(const Vec3 &iX) {
            X = iX;
        }

        virtual void set_R(const double &iR) {
            R = iR;
        }

        virtual void set_c(const double &iC) {
            c = iC;
        };

        /*
         * Simulate single time step
         */

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
        double R;
        double c;
        Vec3 v; // velocity
        Vec3 w; // angular momentum
        Vec3 F; // current force
        Vec3 T; // current torque
        Base *object;
        std::set<functor *> *functors; // functors which calculate forces and torques

        /*
         * helper function which sums up the forces and torques outputed by the
         * functors
         */

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
         * Numerical integration method based on the 4th order Runge-Kutta algorithm
         */

        void do_rk4(double &dT) {

            // calculate first half time step
            Vec3 _v1 = F / c;
            Vec3 _x2 = X + 0.5 * dT * _v1;
            //Vec3 _t1 = _Ii.cwiseProduct(T);
            //Vec3 _w2 = w + 0.5 * dT * _t1;
            //auto _R2 = R + 0.5 * dT * _w2(2);

            auto _k2 = sum_functors(_x2, _v1, R, w);

            // calculate another half time step with new values
            Vec3 _v2 = _k2.first / c;
            Vec3 _x3 = X + 0.5 * dT * _v2;
            //Vec3 _t2 = _Ii.cwiseProduct(_k2.second);
            //Vec3 _w3 = w + 0.5 * dT * _t2;
            //auto _R3 = R + 0.5 * dT * _w3(2);

            auto _k3 = sum_functors(_x3, _v2, R, w);

            // and do it a third time with a full time step
            Vec3 _v3 = _k3.first / c;
            Vec3 _x4 = X + dT * _v3;
            //Vec3 _t3 = _Ii.cwiseProduct(_k3.second);
            //Vec3 _w4 = w + dT * _t3;
            //auto _R4 = R + dT * _w4(2);

            auto _k4 = sum_functors(_x4, _v3, R, w);
            Vec3 _v4 = _k4.first / c;

            // obtain new position and velocity, roation, angular momemntum
            X = X + dT / 6.0 * (v + 2 * _v2 + 2 * _v3 + _v4);
            R = R; //+ dT / 6.0 * (w(2) + 2 * _w2(2) + 2 * _w3(2) + _w4(2));
        }
    };


    /*
     * Base class for all physcial objects
     */

    class Base {
    public:
        Base() :
                solver(nullptr) {};

        virtual ~Base() = default;

        virtual stokes::Solver &Solver() { return *solver; };

    protected:
        stokes::Solver *solver;
    };

};

#endif //CELLFORMATION_STOKESSOLVER_H
