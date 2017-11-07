/*
 * actin implementation for the cell
 *
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
            const Eigen::Vector3d &iStart,
            const Eigen::Vector3d &iTmVelocity,
            const double &iMaxLength,
            const double &iLifeTime,
            const double &iStallingForce,
            const double &iStokesCoeff,
            std::set<stokes::functor *> &iFunctors
    );

    virtual ~actin();

    virtual Eigen::Vector3d &get_tmVelocity();

    virtual bool make_timeStep(double &dT);

protected:
    Eigen::Vector3d tmVelocity; // treadmilling velocity
    Eigen::Vector3d force; // current force vector in actin element
    const double birthTime; // time when object is created
    const double maxLength; // maximum length
    const double lifeTime; // dies after lifetime
    const double stallingForce; // force at which actin doesn't treadmills anymore
    const double stokesCoeff;
};

#endif /* SRC_ACTIN_H_ */
