/*
 * cFac.h
 *
 *  Created on: May 29, 2017
 *      Author: siegbahn
 */

#pragma once

#include "cBaseclasses.h"
#include "globalVars.h"

#ifndef SRC_CFAC_H_
#define SRC_CFAC_H_

class fac : public matrix_base {
public:
    fac(
            sGlobalVars &iGlobals,
            double iRadius,
            double iX,
            double iY
    );

    virtual ~fac();

    virtual void obtain_visualObjs(std::vector<visual_base *> &iVisualObjs);

    virtual void set_radius(double iRadius);

    virtual void set_position(double iX, double iY);

protected:
};

#endif /* SRC_CFAC_H_ */
