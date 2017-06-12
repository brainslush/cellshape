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
    random_container *rndC;
    grid::container *grid;
    double time;
    uint64_t frameNo;
    sSettings settings;

    sGlobalVars() {};

    sGlobalVars(
            mygui::gui *iGuiBase
    ) : guiBase(iGuiBase) {};
};

#endif /* SRC_GLOBALVARS_H_ */
