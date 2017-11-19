

#pragma once

#include <boost/variant.hpp>
#include "base.h"
#include "random.h"
#include "grid.h"
#include "RigidBody.h"
#include "globalVars.h"
#include "stokesSolver.h"

#ifndef __H_CBASECLASSES
#define __H_CBASECLASSES


/*
 * Base class for all physical components.
 * It is the base call for cells and substrates
 */

class components_base : public base {
public:
    explicit components_base(sGlobalVars &iGlobals);

    virtual ~components_base();

    virtual bool &get_canMove();

    virtual bool &get_canColide();

    virtual Eigen::Vector3d &get_responseForce();

    virtual Eigen::Vector3d &get_responseTorque();

    virtual void set_canMove(bool iCanMove);

    virtual void set_canColide(bool iCanColide);

    virtual void add_responseForce(const Eigen::Vector3d &iForce);

    virtual void add_responseTorque(const Eigen::Vector3d &iTorque);

    //virtual void make_timeStep(double &dT);

protected:
    bool canMove; // is it a fixed object
    bool canColide; // can this object collide aka does it have physics?
    Eigen::Vector3d responseForce;
    Eigen::Vector3d responseTorque;
    sGlobalVars &globals;
};

/*
 * cell base class which holds all
 */

class filament_base;

// cell base classes
class cell_base : public components_base {
public:
    explicit cell_base(sGlobalVars &iGlobals);

    virtual ~cell_base();

    virtual void register_filament(filament_base *iFilament) {};

    virtual void unregister_filament(filament_base *iFilament) {};

    virtual void unregister_filament(std::set<filament_base *>::iterator iIt) {};

protected:
};

/*
 * base class for all physical
 */

class cellcomponents_base : public components_base, public stokes::Base {
public:
    cellcomponents_base(sGlobalVars &iGlobals, cell_base &iCell);

    virtual ~cellcomponents_base();

protected:
    cell_base &cell;
    Eigen::Vector3d responseForces;
    Eigen::Vector3d responseTorque;
};

// linkers
class linker_base : public cellcomponents_base {
public:
    linker_base(sGlobalVars &iGlobals, cell_base &iCell);

    virtual ~linker_base();

    virtual std::set<cellcomponents_base *> &get_connectedComponents();

    virtual void add_connectedComponent(cellcomponents_base *iComponent);

    virtual void remove_connectedComponent(cellcomponents_base *iComponent);

    virtual void make_timeStep(const double& dT);

protected:
    std::set<cellcomponents_base *> connectedComponents;
};

// filaments
class filament_base : public cellcomponents_base {
public:
    filament_base(sGlobalVars &iGlobals, cell_base &iCell);

    virtual ~filament_base();

    virtual void set_positions(double iX1, double iY1, double iX2, double iY2);

    virtual std::set<linker_base *> &get_connectedLinkers();

    virtual void add_connectedLinker(linker_base *iLinker);

    virtual void remove_connectedLinker(linker_base *iLinker);

    virtual void obtain_visualObjs(std::vector<visual_base *> &iVisualObjs);

    virtual bool make_timeStep(double &dT);

protected:
    std::set<linker_base *> connectedLinkers;
};

// volume
class volume_base : public cellcomponents_base {
public:
    volume_base(sGlobalVars &iGlobals, cell_base &iCell);

    virtual ~volume_base();

    virtual void make_timeStep(double &dT);

protected:
};

// membrane parts
class membrane_part_base : public cellcomponents_base {
public:
    membrane_part_base(
            sGlobalVars &iGlobals,
            cell_base &iCell,
            double iX1, double iY1,
            double iX2, double iY2
    );

    membrane_part_base(
            sGlobalVars &iGlobals,
            cell_base &iCell
    );

    virtual ~membrane_part_base();

    virtual std::pair<membrane_part_base *, membrane_part_base *> &get_neighbours();

    virtual double get_length();

    virtual std::pair<Eigen::Vector3d *, Eigen::Vector3d *> &get_sharedPositions();

    virtual std::set<linker_base *> &get_connectedLinkers();

    virtual void add_connectedLinker(linker_base *iLinker);

    virtual void remove_connectedLinker(linker_base *iLinker);

    virtual void set_neighbours(const std::pair<membrane_part_base *,membrane_part_base *> &iNeighbours);

    virtual void set_sharedPositions(const std::pair<Eigen::Vector3d *, Eigen::Vector3d *> &iSharedPos);

    virtual Eigen::Vector3d calc_dirVector(Eigen::Vector3d *iPoint);

protected:
    std::pair<membrane_part_base *, membrane_part_base *> neighbours;
    std::pair<Eigen::Vector3d *, Eigen::Vector3d *> sharedPositions;
    std::set<linker_base *> connectedLinkers;
};

// matrix components base class
class matrixcomponents_base : public components_base {
public:
    explicit matrixcomponents_base(sGlobalVars &iGlobals);

    virtual ~matrixcomponents_base();

protected:
};

// fac base class
class fac_base : public matrixcomponents_base {
public:
    explicit fac_base(sGlobalVars &iGlobals);

    virtual ~fac_base();
};

// surface border base class
class surface_border_base : public matrixcomponents_base {
public:
    explicit surface_border_base(sGlobalVars &iGlobals);

    virtual ~surface_border_base();
};

// surface base class
class surface_base : public matrixcomponents_base {
public:
    explicit surface_base(sGlobalVars &iGlobals);

    virtual ~surface_base();
};

#endif
