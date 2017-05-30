/*
 * Actin.cpp
 *
 *  Created on: May 29, 2017
 *      Author: siegbahn
 */

#include "cActin.h"

actin::actin(
    sGlobalVars& iGlobals,
    cell_base& iCell,
    Eigen::Vector3d iStart,
    Eigen::Vector3d iTmVelocity,
    double iMaxLength,
    double iLifeTime,
    double iStallingForce
):
    filament_base(iGlobals,iCell),
    tmVelocity(iTmVelocity),
    birthTime(iGlobals.time),
    maxLength(iMaxLength),
    lifeTime(iLifeTime),
    stallingForce(iStallingForce)
{
    associatedVisualObj->set_color(0,0,1,1);
    associatedVisualObj->set_fillColor(0,1,0,1);
    positions.clear();
    positions.push_back(iStart);
    positions.push_back(iStart);
    tail = NULL;
    forceF = new functor_actin_force(this);
    torqueF = new functor_actin_torque(this);
    // test variables, remove them later

    rigidBody = new physic::RigidBody3d(
        Eigen::Vector3d(0,0,0),
        Eigen::Quaterniond(0,1,0,0),
        Eigen::Matrix3d(),
        0.1,
        0.1,
        forceF,
        torqueF
    );
}
actin::~actin() {
    if(tail != NULL) {
        delete tail;
    }
    delete forceF;
    delete torqueF;
    delete rigidBody;
}
void actin::update_force() {
    /*if (!force.isUpdated()) {
        if (tail != NULL) {
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
void actin::make_timeStep(double& dT) {
    // destroy if it exceeds life time
    if (birthTime + lifeTime < globals.time) {
        cell.destory_filament(this);
    } else {
        positions[1] = positions[1] + tmVelocity * globals.settings.deltaT;
    }
}
/***************************
 * actin functors
 ***************************/
functor_actin_force::functor_actin_force(
    actin* iFillament
):
    fillament(iFillament)
{

}
functor_actin_force::~functor_actin_force(){

}
Eigen::Vector3d functor_actin_force::calc(
    Eigen::Vector3d& X,
    Eigen::Vector3d& v,
    Eigen::Quaterniond& R,
    Eigen::Vector3d& L
){
    return Eigen::Vector3d(0,0,0);
}
functor_actin_torque::functor_actin_torque(
    actin* iFillament
):
    fillament(iFillament)
{

}
functor_actin_torque::~functor_actin_torque(){

}
Eigen::Vector3d functor_actin_torque::calc (
    Eigen::Vector3d& X,
    Eigen::Vector3d& v,
    Eigen::Quaterniond& R,
    Eigen::Vector3d& L
){
    return Eigen::Vector3d(0,0,0);
}
