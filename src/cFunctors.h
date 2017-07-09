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

    class filamentCollision : public physic::functor {
    public:
        filamentCollision(mygui::gui *&iGui);

        virtual ~filamentCollision();

        virtual std::pair<Eigen::Vector3d, Eigen::Vector3d> calc(
                Eigen::Vector3d &X,
                Eigen::Vector3d &v,
                Eigen::Quaterniond &R,
                Eigen::Vector3d &L
        );

    protected:
        mygui::group *guiGroup;
        bool &activated;
    };

    class dampening : public physic::functor {
    public:
        dampening(mygui::gui *&iGui);

        virtual ~dampening();

        virtual std::pair<Eigen::Vector3d, Eigen::Vector3d> calc(
                Eigen::Vector3d &X,
                Eigen::Vector3d &v,
                Eigen::Quaterniond &R,
                Eigen::Vector3d &L
        );

    protected:
        mygui::group *guiGroup;
        bool &activated;
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
