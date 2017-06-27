/*
 * Actin.cpp
 *
 *  Created on: May 29, 2017
 *      Author: siegbahn
 */

#include "cActin.h"

actin::actin(
        sGlobalVars &iGlobals,
        cell_base &iCell,
        Eigen::Vector3d iStart,
        Eigen::Vector3d iTmVelocity,
        double iMaxLength,
        double iLifeTime,
        double iStallingForce,
        std::set<physic::functor *> &iFunctors
) :
        filament_base(iGlobals, iCell),
        tmVelocity(iTmVelocity),
        birthTime(iGlobals.time),
        maxLength(iMaxLength),
        lifeTime(iLifeTime),
        stallingForce(iStallingForce) {
    associatedVisualObj->set_color(0, 0, 1, 1);
    associatedVisualObj->set_fillColor(0, 1, 0, 1);
    positions.clear();
    positions.push_back(iStart);
    positions.push_back(iStart);
    // test variables, remove them later
    double mass = 0.1;
    double length = (positions[0] - positions[1]).norm();
    double I = 0.83333333 * mass * length * length;
    Eigen::Matrix3d mI;
    mI <<   I,0,0,
            0,I,0,
            0,0,0;
    Eigen::Matrix3d mI2 = mI.inverse();
    Eigen::Vector3d mI3 = mI2.diagonal();
    rigidBody = new physic::RigidBody3d(
            Eigen::Vector3d(0, 0, 0),
            Eigen::Quaterniond(0, 0, 0, 1),
            mI,
            0.1,
            0.1,
            iFunctors
    );
}

actin::~actin() {
    delete rigidBody;
    rigidBody = nullptr;
}

void actin::update_force() {
    /*if (!force.isUpdated()) {
        if (tail) {
            force += tail->get_force();
        }
        for (auto& it : connectedCrosslinkers) {
            force += it->get_force();
        }

        force.set_updated(true);
    }
    */
}

Eigen::Vector3d actin::get_force() {
    update_force();
    return force;
}

bool actin::make_timeStep(double &dT) {
    // destroy if it exceeds life time
    if (birthTime + lifeTime < globals.time) {
        return true;
    } else {
        double length = (positions[0]-positions[1]).norm();
        if (length < maxLength) {
            positions[1] = positions[1] + globals.settings.deltaT * tmVelocity;
        } else {
            positions[0] = positions[0] + globals.settings.deltaT * tmVelocity;
            positions[1] = positions[1] + globals.settings.deltaT * tmVelocity;
        }
        if (length > std::numeric_limits<double>::min()) {
            rigidBody->do_timeStep(dT,this);
        }
        return false;
    }
}