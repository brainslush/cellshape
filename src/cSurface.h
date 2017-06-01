/*
 * cSurface.h
 *
 *  Created on: May 29, 2017
 *      Author: siegbahn
 */

#include "cBaseclasses.h"
#include "globalVars.h"
#include "random.h"
#include "extIncludes.h"
#include "cFac.h"

#ifndef SRC_CSURFACE_H_
#define SRC_CSURFACE_H_

class simple_surface;
class surface_border : public matrix_base {
public:
    surface_border(
        sGlobalVars& iGlobals,
        simple_surface* iSurface,
        Eigen::Vector3d iStart,
        Eigen::Vector3d iEnd
    );
    virtual ~surface_border();
    virtual void obtain_visualObjs(std::vector<visual_base*>& iVisualObjs);
protected:
    simple_surface* surface;
};

class simple_surface : public matrix_base {
public:
    simple_surface(
        sGlobalVars& iGlobals,
        double iSideLength
    );
    virtual ~simple_surface();
    virtual void obtain_visualObjs(std::vector<visual_base*>& iVisualObjs);
    virtual void create_facs(unsigned iType, unsigned long long iCount, double iRadius);
protected:
    double sideLength;
    std::vector<surface_border*> borders;
    std::vector<fac*> facs;
};

#endif /* SRC_CSURFACE_H_ */