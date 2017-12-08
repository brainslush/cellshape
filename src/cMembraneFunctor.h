//
// Created by brainslush on 26/11/17.
//

#pragma once

#include "cMembrane.h"
#include "cLinker.h"
#include "cCell.h"

#ifndef CELLFORMATION_CMEMBRANEFUNCTOR_H
#define CELLFORMATION_CMEMBRANEFUNCTOR_H


class cell;

/*
 * functor which handles "membrane" creation
 */

class functor_cell_membraneCreation : public functor_membrane_base {
public:
    explicit functor_cell_membraneCreation(sGlobalVars &iGlobals);

    virtual ~functor_cell_membraneCreation();

    virtual void setup(cell &iCell);

    virtual membrane_part_base *split(
            cell_base *iCell,
            membrane_part_base *iMembrane,
            const Eigen::Vector3d &iPos,
            linker_base *iLinker
    );

    virtual void merge(
            cell_base *iCell
    );

    linker_base *find_sharedLinker(membrane_part *iPartA, membrane_part *iPartB);

    membrane_part_base *mergeChainFront(cell_base *iCell, membrane_part_base *iMembrane);

    membrane_part_base *mergeChainBack(cell_base *iCell, membrane_part_base *iMembrane);

    membrane_part_base *mergeChain(cell_base *iCell, membrane_part_base *iMembrane);

    virtual double get_length(cell_base *iCell);

    virtual void make_timeStep(double &dT, cell *iCell);

protected:
    virtual void update_length(cell_base *iCell);

    double length;
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


#endif //CELLFORMATION_CMEMBRANEFUNCTOR_H
