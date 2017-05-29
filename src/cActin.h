/*
 * Actin.h
 *
 *  Created on: May 29, 2017
 *      Author: siegbahn
 */

#include "cBaseclasses.h"
#include "globalVars.h"
#include "extIncludes.h"

#ifndef SRC_CACTIN_H_
#define SRC_CACTIN_H_

class functor_actin_force;
class functor_actin_torque;
class actin : public fillament_base {
public:
    actin(
        sGlobalVars& iGlobals,
        cell_base& iCell,
        Eigen::Vector3d iStart,
        Eigen::Vector3d iTmVelocity,
        double iMaxLength,
        double iLifeTime,
        double iStallingForce
    );
    virtual ~actin();
    virtual void update_force();
    virtual Eigen::Vector3d get_force();
    virtual void make_timeStep(double& dT);
protected:
    functor_actin_force* forceF;
    functor_actin_torque* torqueF;
    physic::RigidBody3d* rigidBody;
    Eigen::Vector3d tmVelocity; // treadmilling velocity
    Eigen::Vector3d force; // current force vector in actin element
    const double birthTime; // time when object is created
    const double maxLength; // maximum length
    const double lifeTime; // dies after lifetime
    const double stallingForce; // force at which actin doesn't treadmills anymore
    actin* tail;
};
class functor_actin_force: public physic::functor {
public:
    functor_actin_force (actin* iFillament);
    virtual ~functor_actin_force ();
    virtual Eigen::Vector3d calc (
        Eigen::Vector3d& X,
        Eigen::Vector3d& v,
        Eigen::Quaterniond& R,
        Eigen::Vector3d& L
    );
protected:
    actin* fillament;
};
class functor_actin_torque: public physic::functor {
public:
    functor_actin_torque (actin* iFillament);
    virtual ~functor_actin_torque ();
    virtual Eigen::Vector3d calc (
        Eigen::Vector3d& X,
        Eigen::Vector3d& v,
        Eigen::Quaterniond& R,
        Eigen::Vector3d& L
    );
protected:
    actin* fillament;
};

#endif /* SRC_ACTIN_H_ */
