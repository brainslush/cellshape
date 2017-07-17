//
// Created by brainslush on 19.06.17.
//

#pragma once

#include "utility"
#include "RigidBody.h"
#include "gui.h"
#include "cBaseclasses.h"

#ifndef SRC_CFUNCTORS_H_
#define SRC_CFUNCTORS_H_

namespace functor {

    class ffFriction : public physic::functor {
    public:
        ffFriction(mygui::gui *&iGui);

        virtual ~ffFriction();

        virtual std::pair<Eigen::Vector3d, Eigen::Vector3d> calc(
                Eigen::Vector3d &X,
                Eigen::Vector3d &v,
                Eigen::Quaterniond &R,
                Eigen::Vector3d &L,
                physic::RigidBody3d &rigidBody,
                filament_base &filament
        );

    protected:
        mygui::group *guiGroup;
        bool &activated;
        double &frictionCoeff;
        double &angleCoeff;
    };

    class fmCollision : public physic::functor {
    public:
        fmCollision(mygui::gui *&iGui);

        virtual ~fmCollision();

        virtual std::pair<Eigen::Vector3d, Eigen::Vector3d> calc(
                Eigen::Vector3d &X,
                Eigen::Vector3d &v,
                Eigen::Quaterniond &R,
                Eigen::Vector3d &L,
                physic::RigidBody3d &rigidBody,
                filament_base &filament
        );
    protected:
        mygui::group *guiGroup;
        bool &activated;
    };

    class fViscosity : public physic::functor {
    public:
        fViscosity(mygui::gui *&iGui);

        virtual ~fViscosity();

        virtual std::pair<Eigen::Vector3d, Eigen::Vector3d> calc(
                Eigen::Vector3d &X,
                Eigen::Vector3d &v,
                Eigen::Quaterniond &R,
                Eigen::Vector3d &L,
                physic::RigidBody3d &rigidBody
        );

    protected:
        mygui::group *guiGroup;
        bool &activated;
        double &factor;
    };

    class membraneSpring : public physic::functor {
    public:
        membraneSpring(mygui::gui *&iGui);

        virtual ~membraneSpring();

        virtual std::pair<Eigen::Vector3d, Eigen::Vector3d> calc(
                Eigen::Vector3d &X,
                Eigen::Vector3d &v,
                Eigen::Quaterniond &R,
                Eigen::Vector3d &L,
                physic::RigidBody3d &rigidBody,
                membrane_part_base *cell
        );

    protected:
        mygui::group *guiGroup;
        bool &activated;
        double &kStretch;
        double &kBend;
    };
};

#endif // SRC_CFUNCTORS_H_
