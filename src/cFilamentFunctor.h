// auf nem Geburtstag und muss hier schon versuchen heimlich ein paar runen zu verbessern ￼
// Created by brainslush on 26/11/17.
//

#pragma once

#include "cActin.h"
#include "cMembrane.h"
#include "cCell.h"

#ifndef CELLFORMATION_CFILAMENTFUNCTOR_H
#define CELLFORMATION_CFILAMENTFUNCTOR_H


class cell;

/*
 * functor which handles filament creation
 */

class functor_cell_filamentCreation : public functor_filament_base {
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
    bool &randomAngle;
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

#endif //CELLFORMATION_CFILAMENTFUNCTOR_H
