/*
 * cCell.h
 *
 *  Created on: May 29, 2017
 *      Author: siegbahn
 */

#pragma once

#include "random.h"
#include "cFilamentFunctor.h"
#include "cMembraneFunctor.h"
#include "cLinkerFunctor.h"

#ifndef SRC_CCELL_H_
#define SRC_CCELL_H_

/*
 * simple cell object which holds all necessary classes and uses functors
 * to define the creation process of the filaments
 */

class functor_cell_membraneCreation;
class functor_cell_filamentCreation;
class functor_cell_linkerCreation;

class cell : public cell_base {
public:

    cell(
            sGlobalVars &iGlobals,
            functor_cell_membraneCreation *iMembraneF,
            functor_cell_filamentCreation *iFilamentF,
            functor_cell_linkerCreation *iLinkerF
    );

    virtual ~cell();

    virtual std::set<filament_base *> &get_filaments();

    virtual std::set<volume_base *> &get_volumes();

    virtual functor_filament_base *get_filamentFunctor();

    virtual functor_membrane_base *get_membraneFunctor();

    virtual functor_linker_base *get_linkerFunctor();

    virtual double &get_x();

    virtual double &get_y();

    virtual void set_filamentCreationFunctor(functor_cell_filamentCreation *iFunctor);

    virtual void obtain_visualObjs(std::vector<visual_base *> &iVisualObjs);

    virtual void register_filament(filament_base *iFilament);

    virtual void register_linker(linker_base *iLinker);

    virtual void unregister_filament(filament_base *iFilament);

    virtual void unregister_linker(linker_base *iLinker);

    virtual void reset();

    virtual void make_timeStep(double &dT);

protected:
    mygui::group *guiGroup;
    double &x;
    double &y;

    std::set<filament_base *> filaments;
    std::set<volume_base *> volumes;
    std::set<linker_base *> linkers;
    functor_cell_filamentCreation *filamentF;
    functor_cell_membraneCreation *membraneF;
    functor_cell_linkerCreation *linkerF;
};

#endif /* SRC_CCELL_H_ */
