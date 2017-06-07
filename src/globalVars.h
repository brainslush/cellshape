/*
 * globalVars.h
 *
 *  Created on: May 29, 2017
 *      Author: siegbahn
 */

#pragma once

#include "settings.h"
#include "gui.h"
#include "random.h"
#include "grid.h"

#ifndef SRC_GLOBALVARS_H_
#define SRC_GLOBALVARS_H_

struct sGlobalVars {
    mygui::gui *guiBase;
    grid_base *grid;
    random_container *rndC;
    double time;
    uint64_t frameNo;
    sSettings settings;
};

#endif /* SRC_GLOBALVARS_H_ */
