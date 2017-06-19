//
// Created by brainslush on 19.06.17.
//

#pragma once

#include "RigidBody.h"
#include "gui.h"
#include "cBaseclasses.h"


#ifndef SRC_CFUNCTORS_H_
#define SRC_CFUNCTORS_H_

namespace functor {

    class FrictionForce : public physic::functor {
    public:
        FrictionForce(functor_cell_filamentCreation& fuc,filament_base iFilament) {

        };

        virtual ~functor_actin_force();

        virtual Eigen::Vector3d calc(
                Eigen::Vector3d &X,
                Eigen::Vector3d &v,
                Eigen::Quaterniond &R,
                Eigen::Vector3d &L
        );

    protected:
        bool &activated;
    };

    class torque : public physic::functor {
    public:
        torque();

        virtual ~functor_actin_torque();

        virtual Eigen::Vector3d calc(
                Eigen::Vector3d &X,
                Eigen::Vector3d &v,
                Eigen::Quaterniond &R,
                Eigen::Vector3d &L
        );

    protected:
        bool &activated;
    };

}

#endif // SRC_CFUNCTORS_H_
