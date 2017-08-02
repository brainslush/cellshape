/*
 * Actin.h
 *
 *  Created on: May 29, 2017
 *      Author: siegbahn
 */

#pragma once

#include "cBaseclasses.h"
#include "globalVars.h"

#ifndef SRC_CACTIN_H_
#define SRC_CACTIN_H_

class actin : public filament_base {
public:
    actin(
            sGlobalVars &iGlobals,
            cell_base &iCell,
            Eigen::Vector3d iStart,
            Eigen::Vector3d iTmVelocity,
            double iMaxLength,
            double iLifeTime,
            double iStallingForce,
            std::set<physic::functor *> &iFunctors
    );

    virtual ~actin();

    virtual void update_force();

    virtual Eigen::Vector3d get_force();

    virtual bool make_timeStep(double &dT);

protected:
    Eigen::Vector3d tmVelocity; // treadmilling velocity
    Eigen::Vector3d force; // current force vector in actin element
    const double birthTime; // time when object is created
    const double maxLength; // maximum length
    const double lifeTime; // dies after lifetime
    const double stallingForce; // force at which actin doesn't treadmills anymore
};

// add class to the registrar
registrar::n::registerType<filament_base,actin>();

#endif /* SRC_ACTIN_H_ */
