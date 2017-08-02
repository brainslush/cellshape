#pragma once

#include <boost/variant.hpp>
#include "base.h"
#include "random.h"
#include "grid.h"
#include "RigidBody.h"
#include "globalVars.h"


#ifndef __H_CBASECLASSES
#define __H_CBASECLASSES


/*
 * Base function for all physical components.
 * It is the base call for cells and substrates
 */

class components_base : public base {
public:
    components_base(sGlobalVars &iGlobals);

    virtual ~components_base();

    virtual bool &get_canMove();

    virtual bool &get_canColide();

    virtual Eigen::Vector3d &get_responseForce();

    virtual Eigen::Vector3d &get_responseTorque();

    virtual void set_canMove(bool iCanMove);

    virtual void set_canColide(bool iCanColide);

    virtual void add_responseForce(Eigen::Vector3d iForce);

    virtual void add_responseTorque(Eigen::Vector3d iTorque);

    //virtual void make_timeStep(double &dT);

protected:
    bool canMove; // is it a fixed object
    bool canColide; // can this object collide aka does it have physics?
    Eigen::Vector3d responseForce;
    Eigen::Vector3d responseTorque;
    sGlobalVars &globals;
};

// add class to the registrar
registrar::n::registerType<base,components_base>();

class filament_base;

// cell base classes
class cell_base : public components_base {
public:
    cell_base(sGlobalVars &iGlobals);

    virtual ~cell_base();

    virtual void register_filament(filament_base *iFilament) {};

    virtual void unregister_filament(filament_base *iFilament) {};

    virtual void unregister_filament(std::set<filament_base *>::iterator iIt) {};

protected:
};

// add class to the registrar
registrar::n::registerType<components_base,cell_base>();

class cellcomponents_base : public components_base {
public:
    cellcomponents_base(sGlobalVars &iGlobals, cell_base &iCell);

    virtual ~cellcomponents_base();

    virtual physic::RigidBody3d &get_rigidBody();

protected:
    cell_base &cell;
    physic::RigidBody3d rigidBody;
    Eigen::Vector3d responseForces;
    Eigen::Vector3d responseTorque;
};

// add class to the registrar
registrar::n::registerType<components_base,cellcomponents_base>();

// crosslinkers
class crosslinker_base : public cellcomponents_base {
public:
    crosslinker_base(sGlobalVars &iGlobals, cell_base &iCell);

    virtual ~crosslinker_base();

    virtual std::set<filament_base *> &get_connectedFilaments();

    virtual Eigen::Vector3d &get_force(unsigned long long iTimeStamp);

    virtual void add_connectedFilament(filament_base *iFilament);

    virtual void remove_connectedFilament(filament_base *iFilament);

protected:
    std::set<filament_base *> connectedFilaments;
    Eigen::Vector3d force;
};

// add class to the registrar
registrar::n::registerType<cellcomponents_base,crosslinker_base>();

// filaments
class filament_base : public cellcomponents_base {
public:
    filament_base(sGlobalVars &iGlobals, cell_base &iCell);

    virtual ~filament_base();

    virtual void set_positions(double iX1, double iY1, double iX2, double iY2);

    virtual void add_connectedCrosslinker(crosslinker_base *iCrosslinker);

    virtual void remove_connectedCrosslinker(crosslinker_base *iCrosslinker);

    virtual void obtain_visualObjs(std::vector<visual_base *> &iVisualObjs);

    virtual bool make_timeStep(double &dT);

protected:
    std::set<crosslinker_base *> connectedCrosslinkers;
    double length;
};

// add class to the registrar
registrar::n::registerType<cellcomponents_base,filament_base>();

// volume
class volume_base : public cellcomponents_base {
public:
    volume_base(sGlobalVars &iGlobals, cell_base &iCell);

    virtual ~volume_base();

    virtual void make_timeStep(double &dT);

protected:
};

// add class to the registrar
registrar::n::registerType<cellcomponents_base,volume_base>();

// membrane parts
class membrane_part_base : public cellcomponents_base {
public:
    membrane_part_base(
            sGlobalVars &iGlobals,
            cell_base &iCell,
            double iX1, double iY1,
            double iX2, double iY2
    );

    virtual ~membrane_part_base();

    virtual std::pair<membrane_part_base *, membrane_part_base *> &get_neighbours();

    virtual double get_length();

    virtual double &get_restLength();

    virtual std::pair<Eigen::Vector3d *, Eigen::Vector3d *> &get_sharedPositions();

    virtual void set_neighbours(std::pair<membrane_part_base *, membrane_part_base *> iNeighbours);

    virtual Eigen::Vector3d calc_dirVector(Eigen::Vector3d *iPoint);

protected:
    std::pair<membrane_part_base *, membrane_part_base *> neighbours;
    std::pair<Eigen::Vector3d *, Eigen::Vector3d *> sharedPositions;
    double restLength;
};

// add class to the registrar
registrar::n::registerType<cellcomponents_base,membrane_part_base>();

// matrix components base class
class matrixcomponents_base : public components_base {
public:
    matrixcomponents_base(sGlobalVars &iGlobals);

    virtual ~matrix_base();

protected:
};

// add class to the registrar
registrar::n::registerType<components_base,matrixcomponents_base>();

// fac base class
class fac_base : public matrixcomponents_base {
public:
    fac_base(sGlobalVars &iGlobals);

    virtual ~fac_base();
};

// add class to the registrar
registrar::n::registerType<matrixcomponents_base,fac_base>();

// surface border base class
class surface_border_base : public matrixcomponents_base {
public:
    surface_border_base(sGlobalVars &iGlobals);

    virtual ~surface_border_base();
}

// add class to the registrar
registrar::n::registerType<matrixcomponents_base,surface_border_base>();

// surface base class
class surface_base : public matrixcomponents_base {
public:
    surface_base(sGlobalVars &iGlobals);

    virtual ~surface_base();
};

// add class to the registrar
registrar::n::registerType<matrixcomponents_base,surface_base>();

#endif
