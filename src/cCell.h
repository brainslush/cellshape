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
#include "cLinker.h"

#ifndef SRC_CCELL_H_
#define SRC_CCELL_H_

class functor_cell_filamentCreation;

class functor_cell_membraneCreation;

/*
 * simple cell object which holds all necessary classes and uses functors
 * to define the creation process of the filaments
 */

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

    virtual void register_linker(linker_base *iLinker);

    virtual void unregister_filament(filament_base *iFilament);

    virtual void unregister_filament(std::set<filament_base *>::iterator iIt);

    virtual void unregister_filament(linker_base *iLinker);

    virtual void reset();

    virtual void make_timeStep(double &dT);

protected:
    mygui::group *guiGroup;
    double &x;
    double &y;

    std::set<membrane_container *> membranes;
    std::set<filament_base *> filaments;
    std::set<volume_base *> volumes;
    std::set<linker_base *> linkers;
    functor_cell_filamentCreation *filamentF;
    functor_cell_membraneCreation *membraneF;
};

/*
 * base class for all cell functors
 */

class functor_cell_base {
public:
    functor_cell_base(
            sGlobalVars &iGlobals,
            std::string iName,
            std::string iFunctorGroupName
    );

    virtual ~functor_cell_base();

    virtual void register_functor(stokes::functor *iFunctor);

    virtual mygui::gui *&get_guiFunctor();

protected:
    sGlobalVars &globals;
    mygui::group *guiGroup;
    std::set<stokes::functor *> functors;
    mygui::gui *guiFunctorGroup;
};

/*
 * functor which handles filament creation
 */

class functor_cell_filamentCreation : public functor_cell_base {
public:
    explicit functor_cell_filamentCreation(
            sGlobalVars &iGlobals
    );

    virtual ~functor_cell_filamentCreation();

    virtual void setup(cell &iCell);

    virtual void make_timeStep(double &dT, cell *iCell);

protected:
    virtual filament_base *create_filament(cell &iCell);

    virtual std::tuple<membrane_part_base *, Eigen::Vector3d, Eigen::Vector3d> find_creationParameters(cell &iCell);

    virtual double find_maxLength(cell &iCell);

    virtual double find_lifeTime(cell &iCell);

    virtual double find_stallingForce(cell &iCell);

    virtual double find_stokesCoeff(cell &iCell);

    random_dist *randomReal;
    unsigned &maxCount;
    bool& randomAngle;
    double &maxTMV;
    bool &constTMV;
    double &maxLength;
    bool &infLength;
    double &maxLifeTime;
    bool &infLifeTime;
    double &maxStallingForce;
    double &bound1StokesCoeff;
    double &bound2StokesCoeff;
    bool &constStokesCoeff;
};

/*
 * functor which handles "membrane" creation
 */

class functor_cell_membraneCreation : public functor_cell_base {
public:
    explicit functor_cell_membraneCreation(sGlobalVars &iGlobals);

    virtual ~functor_cell_membraneCreation();

    virtual void setup(cell &iCell);

protected:
    double &radius;
    unsigned &resolution;
};

/*
 * functor which handles creation of membrane made of arc circles
 */

class functor_cell_arcMembraneCreation : public functor_cell_membraneCreation {
public:
    explicit functor_cell_arcMembraneCreation(sGlobalVars &iGlobals);

    virtual ~functor_cell_arcMembraneCreation();

    virtual void setup(cell &iCell);
};

#endif /* SRC_CCELL_H_ */
