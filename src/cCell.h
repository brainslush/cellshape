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

class functor_cell_membraneCreation;

class cell : public cell_base {
public:

    cell(
            sGlobalVars &iGlobals,
            functor_cell_membraneCreation *iMembraneF,
            functor_cell_filamentCreation *iFilamentF
    );

    virtual ~cell();

    virtual std::set<membrane_container *> &get_membranes();

    virtual std::set<filament_base *> &get_filaments();

    virtual std::set<volume_base *> &get_volumes();

    virtual double &get_x();

    virtual double &get_y();

    virtual void set_filamentCreationFunctor(functor_cell_filamentCreation *iFunctor);

    virtual void obtain_visualObjs(std::vector<visual_base *> &iVisualObjs);

    virtual void register_filament(filament_base *iFilament);

    virtual void unregister_filament(filament_base *iFilament);

    virtual void unregister_filament(std::set<filament_base *>::iterator iIt);

    virtual void reset();

    virtual void make_timeStep(double &dT);

protected:
    mygui::group *guiGroup;
    double &x;
    double &y;

    std::set<membrane_container *> membranes;
    std::set<filament_base *> filaments;
    std::set<volume_base *> volumes;
    functor_cell_filamentCreation *filamentF;
    functor_cell_membraneCreation *membraneF;
};

// base class for cell functor
class functor_cell_base {
public:
    functor_cell_base(
            sGlobalVars &iGlobals,
            std::string iName,
            std::string iFunctorGroupName
    );

    virtual ~functor_cell_base();

    virtual void register_functor(physic::functor *iFunctor);

    virtual mygui::gui *&get_guiFunctor();

protected:
    sGlobalVars &globals;
    mygui::group *guiGroup;
    std::set<physic::functor *> functors;
    mygui::gui *guiFunctorGroup;
};

class functor_cell_filamentCreation : public functor_cell_base {
public:
    functor_cell_filamentCreation(
            sGlobalVars &iGlobals
    );

    virtual ~functor_cell_filamentCreation();

    virtual void setup(cell &iCell);

    virtual void make_timeStep(double &dT, cell *iCell);

protected:
    virtual filament_base *create_filament(cell &iCell);

    virtual pair<Eigen::Vector3d, membrane_part *> find_creationPosition(cell &iCell);

    virtual Eigen::Vector3d find_tmVelocity(cell &iCell, membrane_part &iMembrane);

    virtual double find_maxLength(cell &iCell);

    virtual double find_lifeTime(cell &iCell);

    virtual double find_stallingForce(cell &iCell);

    random_dist *randomReal;
    unsigned &maxCount;
    double &maxSpeed;
    double &maxLength;
    double &maxLifeTime;
    double &maxStallingForce;
};

class functor_cell_membraneCreation : public functor_cell_base {
public:
    functor_cell_membraneCreation(sGlobalVars &iGlobals);

    virtual ~functor_cell_membraneCreation();

    virtual void setup(cell &iCell);

protected:
    double &radius;
    unsigned &resolution;
};

#endif /* SRC_CCELL_H_ */
