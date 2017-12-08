

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

    //virtual void make_timeStep(double &dT);

protected:
    sGlobalVars &globals;
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

    virtual std::set<stokes::functor *> &get_functors();

    virtual void register_functor(stokes::functor *iFunctor);

    virtual mygui::gui *&get_guiFunctor();

protected:
    sGlobalVars &globals;
    mygui::group *guiGroup;
    std::set<stokes::functor *> functors;
    mygui::gui *guiFunctorGroup;
};


/*
 * cell base class which holds all
 */


class linker_base;

class filament_base;

class functor_membrane_base;

class functor_filament_base;

class functor_linker_base;

class membrane_container;

// cell base classes
class cell_base : public components_base {
public:
    explicit cell_base(sGlobalVars &iGlobals);

    virtual ~cell_base();

    virtual functor_filament_base *get_filamentFunctor() { return nullptr; };

    virtual functor_membrane_base *get_membraneFunctor() { return nullptr; };

    virtual functor_linker_base *get_linkerFunctor() { return nullptr; };

    virtual membrane_container *get_membrane() { return membrane; };

    virtual void set_membrane(membrane_container *iMembrane) { membrane = iMembrane; };

    virtual void register_filament(filament_base *iFilament) {};

    virtual void register_linker(linker_base *iLinker) {};

    virtual void unregister_filament(filament_base *iFilament) {};

    virtual void unregister_linker(linker_base *iLinker) {};

protected:
    membrane_container *membrane;
};

/*
 * base class for all simulated components
 */

class cellcomponents_base : public components_base, public stokes::Base {
public:
    cellcomponents_base(sGlobalVars &iGlobals, cell_base &iCell);

    virtual ~cellcomponents_base();

    virtual std::set<linker_base *> &get_connectedLinkers();

    virtual void add_connectedLinker(linker_base *iComponent);

    void remove_connectedLinker(linker_base *iComponent);

protected:
    cell_base &cell;
    std::set<linker_base *> connectedLinkers;
};

// linkers
class linker_base : public components_base {
public:
    linker_base(sGlobalVars &iGlobals, cell_base &iCell);

    virtual ~linker_base();

    virtual std::set<cellcomponents_base *> &get_connectedComponents();

    virtual void set_connectedComponents(const std::set<cellcomponents_base *> &iComponents);

    virtual void add_connectedComponent(cellcomponents_base *iComponent);

    virtual void remove_connectedComponent(cellcomponents_base *iComponent);

    virtual void make_timeStep(const double &dT);

protected:
    cell_base &cell;
    std::set<cellcomponents_base *> connectedComponents;
};

// filaments
class filament_base : public cellcomponents_base {
public:
    filament_base(sGlobalVars &iGlobals, cell_base &iCell);

    virtual ~filament_base();

    virtual void set_positions(double iX1, double iY1, double iX2, double iY2);

    virtual void obtain_visualObjs(std::vector<visual_base *> &iVisualObjs);

    virtual bool make_timeStep(double &dT);

protected:
};

// volume
class volume_base : public cellcomponents_base {
public:
    volume_base(sGlobalVars &iGlobals, cell_base &iCell);

    virtual ~volume_base();

    virtual void make_timeStep(double &dT);

protected:
};

class membrane_part_base;

// membrane container which organizes membrane parts and its creation
class membrane_container : public cellcomponents_base {
public:

    membrane_container(
            sGlobalVars &iGlobals,
            cell_base &iCell
    );

    virtual ~membrane_container();

    virtual membrane_part_base *back();

    virtual membrane_part_base *begin();

    virtual membrane_part_base *end();

    virtual unsigned long long size();

    virtual membrane_part_base *insert_before(membrane_part_base *iPos, membrane_part_base *iPart);

    virtual membrane_part_base *insert_after(membrane_part_base *iPos, membrane_part_base *iPart);

    virtual membrane_part_base *delete_part(membrane_part_base *iPart);

    virtual void obtain_visualObjs(std::vector<visual_base *> &oVisualComponents);

protected:
    double length;

    membrane_part_base *vback;
    membrane_part_base *vbegin;
    membrane_part_base *vend;
    unsigned long long vsize;
};

// membrane parts
class membrane_part_base : public cellcomponents_base {
public:
    membrane_part_base(
            sGlobalVars &iGlobals,
            cell_base &iCell
    );

    virtual ~membrane_part_base();

    virtual double get_length();

    void set_next(membrane_part_base *iN) { vnext = iN; };

    void set_prev(membrane_part_base *iP) { vprev = iP; };

    membrane_part_base *next() {
        return vnext;
    };

    membrane_part_base *prev() {
        return vprev;
    }

    membrane_part_base *itnext() {
        if (this == cell.get_membrane()->back()) {
            return cell.get_membrane()->end();
        }
        return vnext;
    }

protected:
    membrane_part_base *vnext;
    membrane_part_base *vprev;
    membrane_linker_base *vnextLinker;
    membrane_linker_base *vprevLinker;
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

// specific functor base classes
class functor_membrane_base : public functor_cell_base {
public:
    functor_membrane_base(
            sGlobalVars &iGlobals,
            std::string iName,
            std::string iFunctorGroupName
    );

    virtual ~functor_membrane_base();

    virtual membrane_part_base *split(
            cell_base *iCell,
            membrane_part_base *iMembranePart,
            const Eigen::Vector3d &iPos,
            linker_base *iLinker
    );

    virtual void merge(
            cell_base *iCell
    );

    virtual double get_length(cell_base *iCell);

    virtual void make_timeStep(double &dT, membrane_part_base *iMembrane);

protected:
};

class functor_filament_base : public functor_cell_base {
public:
    functor_filament_base(
            sGlobalVars &iGlobals,
            std::string iName,
            std::string iFunctorGroupName
    );

    virtual ~functor_filament_base();

protected:
};

class functor_linker_base : public functor_cell_base {
public:
    functor_linker_base(
            sGlobalVars &iGlobals,
            std::string iName,
            std::string iFunctorGroupName
    );

    virtual ~functor_linker_base();

    virtual linker_base* create_linker(cell_base *iCell, const std::set<cellcomponents_base *> &iConnectedComponents);

    virtual void delete_linker(cell_base *iCell, linker_base* iLinker);
};

#endif
