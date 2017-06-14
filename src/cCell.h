/*
 * cCell.h
 *
 *  Created on: May 29, 2017
 *      Author: siegbahn
 */

#pragma once

#include <set>
#include "cBaseclasses.h"
#include "globalVars.h"
#include "random.h"
#include "cMembrane.h"
#include "cActin.h"

#ifndef SRC_CCELL_H_
#define SRC_CCELL_H_

class functor_cell_filamentCreation;

class cell : public cell_base {
public:

    cell(
            sGlobalVars &iGlobals,
            functor_cell_filamentCreation *iFunctor
    );

    virtual ~cell();

    virtual functor_cell_filamentCreation *&get_filamentCreationFunctor();

    virtual std::set<membrane_base *> &get_membranes();

    virtual std::set<filament_base *> &get_filaments();

    virtual std::set<volume_base *> &get_volumes();

    virtual void set_filamentCreationFunctor(functor_cell_filamentCreation *iFunctor);

    virtual void obtain_visualObjs(std::vector<visual_base *> &iVisualObjs);

    virtual void register_filament(filament_base *iFilament);

    virtual void unregister_filament(filament_base *iFilament);

    virtual void reset();

    virtual void make_timeStep(double &dT);

protected:
    mygui::group *guiGroup;
    double &x;
    double &y;
    double &radius;
    unsigned &resolution;

    std::set<membrane_base *> membranes;
    std::set<filament_base *> filaments;
    std::set<volume_base *> volumes;
    functor_cell_filamentCreation *filamentF;
};

class functor_cell_filamentCreation {
public:
    functor_cell_filamentCreation(
            sGlobalVars &iGlobals
    );

    virtual ~functor_cell_filamentCreation();

    virtual void setup(cell *iCell);

    virtual void make_timeStep(double &dT, cell *iCell);

protected:
    virtual filament_base *create_filament(cell *iCell);

    virtual pair<Eigen::Vector3d, membrane_part *> find_creationPosition(cell *iCell);

    virtual Eigen::Vector3d find_tmVelocity(cell *iCell, membrane_part *iMembrane);

    virtual double find_maxLength(cell *iCell);

    virtual double find_lifeTime(cell *iCell);

    virtual double find_stallingForce(cell *iCell);

    sGlobalVars &globals;
    random_dist *randomReal;
    mygui::group *guiGroup;
    unsigned &maxCount;
    double &maxSpeed;
    double &maxLength;
    double &maxLifeTime;
    double &maxStallingForce;

};

#endif /* SRC_CCELL_H_ */
