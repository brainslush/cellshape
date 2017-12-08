/*
 * cMembrane.h
 *
 *  Created on: May 29, 2017
 *      Author: siegbahn
 */

#pragma once

#include "cBaseclasses.h"
#include "globalVars.h"

#ifndef SRC_CMEMBRANE_H_
#define SRC_CMEMBRANE_H_

// classical membrane element
class membrane_part : public membrane_part_base {
public:
    // 2D membrane part
    membrane_part(
            sGlobalVars &iGlobals,
            cell_base &iCell,
            double iX1, double iY1,
            double iX2, double iY2,
            std::set<stokes::functor *> &iFunctors
    );

    membrane_part(
            sGlobalVars &iGlobals,
            cell_base &iCell,
            const Eigen::Vector3d & iPosS,
            const Eigen::Vector3d & iPosE,
            std::set<stokes::functor *> &iFunctors
    );

    virtual ~membrane_part();

    virtual void set_sharedPositions(const std::pair<Eigen::Vector3d *, Eigen::Vector3d *> &iSharedPos);

    virtual std::pair<Eigen::Vector3d *, Eigen::Vector3d *> &get_sharedPositions();

    virtual Eigen::Vector3d &get_normal();

    virtual double get_length();

    virtual void obtain_visualObjs(std::vector<visual_base *> &iVisualObjs);

    virtual void make_timeStep(const double &dT);

protected:
    Eigen::Vector3d normal;
    std::pair<Eigen::Vector3d *, Eigen::Vector3d *> sharedPositions;

    virtual void update_normal();
};

class arc_membrane_part : public membrane_part_base {
public:
    arc_membrane_part(
            sGlobalVars &iGlobals,
            cell_base &iCell,
            double iX1, double iY1,
            double R,
            double angleB,
            double angleE,
            std::set<stokes::functor *> &iFunctors
    );

    virtual ~arc_membrane_part();

    virtual Eigen::Vector3d get_normal(const double &iDeg);

    virtual double get_length();

    virtual void obtain_visualObjs(std::vector<visual_base *> &iVisualObjs);

    virtual void make_timeStep(const double &dT);

protected:
};

#endif /* SRC_CMEMBRANE_H_ */
