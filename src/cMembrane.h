/*
 * cMembrane.h
 *
 *  Created on: May 29, 2017
 *      Author: siegbahn
 */

#include "cBaseclasses.h"
#include "globalVars.h"
#include "extIncludes.h"

#ifndef SRC_CMEMBRANE_H_
#define SRC_CMEMBRANE_H_

class membrane_part : public cellcomponents_base {
public:
    // 2D membrane part
    membrane_part(
        sGlobalVars& iGlobals, cell_base& iCell,
        double iX1,double iY1,
        double iX2,double iY2
    );
    virtual ~membrane_part();
    virtual double& get_length();
    virtual Eigen::Vector3d& get_normal();
    virtual void obtain_visualObjs(std::vector<visual_base*>& iVisualObjs);
    virtual void set_neighbours(membrane_part& iPartA,membrane_part& iPartB);
    virtual void make_timeStep(double& dT);
protected:
    std::vector<membrane_part*> neighbours;
    double length;
    Eigen::Vector3d normal;
};
class membrane_base : public cellcomponents_base {
public:
    membrane_base(
        sGlobalVars& iGlobals, cell_base& iCell,
        double iX, double iY,
        double iRadius,
        unsigned long long iResolution
    );
    virtual ~membrane_base();
    virtual double& get_area();
    virtual double& get_length();
    virtual std::vector<membrane_part*>& get_parts();
    virtual void obtain_visualObjs(std::vector<visual_base*>& oVisualComponents);
    virtual void make_timeStep(double& dT);
protected:
    double area;
    double length;
    const bool canColide = true;
    std::vector<membrane_part*> parts;
    virtual void update_area();
    virtual void update_length();
};

#endif /* SRC_CMEMBRANE_H_ */