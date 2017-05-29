/*
 * cCell.h
 *
 *  Created on: May 29, 2017
 *      Author: siegbahn
 */

#include "cBaseclasses.h"
#include "globalVars.h"
#include "extIncludes.h"
#include "cMembrane.h"
#include "cActin.h"

#ifndef SRC_CCELL_H_
#define SRC_CCELL_H_

class cell : public cell_base {
public:
    cell(
        sGlobalVars& iGlobals,
        double iX,
        double iY,
        unsigned long long iResolution
    );
    virtual ~cell();
    virtual void obtain_visualObjs(std::vector<visual_base*>& iVisualObjs);
    virtual void create_fillament();
    virtual void destory_fillament(fillament_base* iFillament);
    virtual void make_timeStep(double& dT);
protected:
    double maxFillamentLength;
    std::set<membrane_base*> membranes;
    std::set<fillament_base*> fillaments;
    std::set<volume_base*> volumes;
};

#endif /* SRC_CCELL_H_ */
