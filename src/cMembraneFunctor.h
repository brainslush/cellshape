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

    virtual membrane_linker_base *split(
            cell_base *iCell,
            membrane_part_base *iMembrane,
            const Eigen::Vector3d &iPos
    );

    virtual void merge(
            cell_base *iCell
    );

    linker_base *find_sharedLinker(membrane_part *iPartA, membrane_part *iPartB);

    membrane_part_base *mergeChainFront(cell_base *iCell, membrane_part_base *iMembrane);

    membrane_part_base *mergeChainBack(cell_base *iCell, membrane_part_base *iMembrane);

    membrane_part_base *mergeChain(cell_base *iCell, membrane_part_base *iMembrane);

    virtual double get_length(cell_base *iCell);

    virtual const unsigned &get_resolution() { return resolution; };

    virtual void update_positions(cell_base *iCell);

    virtual void make_timeStep(double &dT, cell *iCell);

protected:
    virtual void update_length(cell_base *iCell);

    double length;
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

class functor_cell_hyperbolicMembraneCreation : public functor_cell_membraneCreation {
public:
    explicit functor_cell_hyperbolicMembraneCreation(sGlobalVars &iGlobals);

    virtual ~functor_cell_hyperbolicMembraneCreation();

    virtual void setup(cell &iCell);

    virtual void update_positions(cell_base *iCell);

    virtual void make_timeStep(double &dT, cell *iCell);
protected:
    virtual std::vector<membrane_linker_base *> membranepeaks(cell &iCell);
    virtual void update_variables();
    double &epsilon;
    double &bp;
    double cutoff;
    double alpha;
    double alpha2;
    double beta;
    double beta2;
    double cotbp;
    double oldEpsilon;
    double oldBp;
};

#endif //CELLFORMATION_CMEMBRANEFUNCTOR_H
