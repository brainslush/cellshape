/*
 * globalVars.h
 *
 *  Created on: May 29, 2017
 *      Author: siegbahn
 */

#include "settings.h"
#include "random.h"

#ifndef SRC_GLOBALVARS_H_
#define SRC_GLOBALVARS_H_

class grid_base;
struct sGlobalVars {
    grid_base* grid;
    random_container* rndC;
    double time;
    uint64_t frameNo;
    sSettings settings;
};

#endif /* SRC_GLOBALVARS_H_ */
