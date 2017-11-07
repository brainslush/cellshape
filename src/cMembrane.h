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

    virtual ~membrane_part();

    virtual Eigen::Vector3d &get_normal();

    virtual void obtain_visualObjs(std::vector<visual_base *> &iVisualObjs);

    virtual void make_timeStep(double &dT);

protected:
    Eigen::Vector3d normal;
};

// membrane container which organizes membrane parts and its creation
class membrane_container : public cellcomponents_base {
public:
    membrane_container(
            sGlobalVars &iGlobals,
            cell_base &iCell
    );

    virtual ~membrane_container();

    virtual double &get_area();

    virtual double &get_length();

    virtual std::vector<membrane_part_base *> &get_parts();

    virtual void obtain_visualObjs(std::vector<visual_base *> &oVisualComponents);

    virtual void make_timeStep(double &dT);

protected:
    double area;
    double length;
    const bool canColide = true;
    std::vector<membrane_part_base *> parts;

    virtual void update_area();

    virtual void update_length();
};

#endif /* SRC_CMEMBRANE_H_ */
