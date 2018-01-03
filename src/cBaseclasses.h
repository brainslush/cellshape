

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


/***************************************************
 * base class for all components
 */

class components_base : public base {
public:
    explicit components_base(sGlobalVars &iGlobals);

    virtual ~components_base();

    virtual const sGlobalVars &get_globals() { return globals; };

protected:
    sGlobalVars &globals;
};

/***************************************************
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

/***************************************************
 * cell base class, contains all cell related
 * elements
 */

class linker_base;

class filament_base;

class functor_membrane_base;

class functor_filament_base;

class functor_linker_base;

class membrane_container;

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

/***************************************************
 * base class for all cell components
 * with a physical body, see stokes class
 * which uses stokes solver for force
 * induced psotion updates
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

/***************************************************
 * linkers base class
 */

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

/***************************************************
 * filaments base class
 */

class membrane_linker_base;

class filament_base : public cellcomponents_base {
public:
    filament_base(sGlobalVars &iGlobals, cell_base &iCell);

    virtual ~filament_base();

    virtual membrane_linker_base *get_connectedMembraneLinker();

    virtual void set_positions(double iX1, double iY1, double iX2, double iY2);

    virtual void set_connectedMembraneLinker(membrane_linker_base *iLinker);

    virtual void obtain_visualObjs(std::vector<visual_base *> &iVisualObjs);

    virtual bool make_timeStep(double &dT);

protected:
    membrane_linker_base *connectedMembraneLinker;
};

/***************************************************
 * volumes base class
 */

class volume_base : public cellcomponents_base {
public:
    volume_base(sGlobalVars &iGlobals, cell_base &iCell);

    virtual ~volume_base();

    virtual void make_timeStep(double &dT);

protected:
};

/***************************************************
 * membrane container base class
 */

class membrane_part_base;

class membrane_container : public cellcomponents_base {
public:

    membrane_container(
            sGlobalVars &iGlobals,
            cell_base &iCell
    );

    virtual ~membrane_container();

    virtual membrane_part_base *back() { return vBack; };

    virtual membrane_part_base *begin() { return vBegin; };

    virtual membrane_part_base *end() { return vEnd; };

    virtual unsigned long long size() { return vSize; };

    virtual membrane_part_base *insert_before(membrane_part_base *iPos, membrane_part_base *iPart);

    virtual membrane_part_base *insert_after(membrane_part_base *iPos, membrane_part_base *iPart);

    virtual membrane_part_base *delete_part(membrane_part_base *iPart);

    virtual void obtain_visualObjs(std::vector<visual_base *> &oVisualComponents);

    virtual void check_integrity(const std::string &iModule);

protected:
    double length;

    membrane_part_base *vBack;
    membrane_part_base *vBegin;
    membrane_part_base *vEnd;
    unsigned long long vSize;
};

/***************************************************
 * membrane linker base class
 * it is different from the regular linker
 * class so it doesn't inherit from it
 */

class membrane_linker_base : public components_base {
public:
    membrane_linker_base(sGlobalVars &iGlobals, cell_base &iCell);

    virtual ~membrane_linker_base();

    virtual membrane_part_base *nextMembrane() { return vNextMembrane; };

    virtual membrane_part_base *prevMembrane() { return vPrevMembrane; }

    virtual membrane_linker_base *nextLinker();

    virtual membrane_linker_base *prevLinker();

    virtual filament_base *connectedFillament() { return vConnectedFilament; };

    virtual Eigen::Vector3d *referencePos() { return vReferencePos; };

    virtual void set_nextMembrane(membrane_part_base *iN) { vNextMembrane = iN; };

    virtual void set_prevMembrane(membrane_part_base *iP) { vPrevMembrane = iP; };

    virtual void set_connectedFilament(filament_base *iF) { vConnectedFilament = iF; };

    virtual void set_referencePosition(Eigen::Vector3d *iR) { vReferencePos = iR; };

    virtual void clear_filament() {
        vReferencePos = nullptr;
        vConnectedFilament = nullptr;
    };

    virtual void clear_nextMembrane() { vNextMembrane = nullptr; };

    virtual void clear_prevMembrane() { vPrevMembrane = nullptr; };

    virtual void obtain_visualObjs(std::vector<visual_base *> &iVisualObjs);

    virtual void make_timeStep(const double &dT);

protected:
    membrane_part_base *vNextMembrane;
    membrane_part_base *vPrevMembrane;
    filament_base *vConnectedFilament;
    Eigen::Vector3d *vReferencePos;
    cell_base &cell;
};

/***************************************************
 * membrane parts base class
 */

class membrane_part_base : public cellcomponents_base {
public:
    membrane_part_base(
            sGlobalVars &iGlobals,
            cell_base &iCell
    );

    virtual ~membrane_part_base();

    virtual double get_length();

    void set_next(membrane_part_base *iN) { vNext = iN; };

    void set_prev(membrane_part_base *iP) { vPrev = iP; };

    void set_nextLinker(membrane_linker_base *iNL) { vNextLinker = iNL; };

    void set_prevLinker(membrane_linker_base *iPL) { vPrevLinker = iPL; };

    membrane_part_base *next() { return vNext; };

    membrane_part_base *prev() { return vPrev; };

    membrane_linker_base *nextLinker() { return vNextLinker; };

    membrane_linker_base *prevLinker() { return vPrevLinker; };

    membrane_part_base *itnext() {
        if (!vNext && !vPrev) {
            return this;
        }
        if (this == cell.get_membrane()->back()) {
            return cell.get_membrane()->end();
        }
        return vNext;
    }

    virtual membrane_linker_base *createLinker() {
        return new membrane_linker_base(globals, cell);
    }

protected:
    membrane_part_base *vNext;
    membrane_part_base *vPrev;
    membrane_linker_base *vNextLinker;
    membrane_linker_base *vPrevLinker;
};

/***************************************************
 * cellular matrix base class
 */

class matrixcomponents_base : public components_base {
public:
    explicit matrixcomponents_base(sGlobalVars &iGlobals);

    virtual ~matrixcomponents_base();

protected:
};

/***************************************************
 * fac base class (deprecated)
 */

class fac_base : public matrixcomponents_base {
public:
    explicit fac_base(sGlobalVars &iGlobals);

    virtual ~fac_base();
};

/***************************************************
 * surface border base class
 */

class surface_border_base : public matrixcomponents_base {
public:
    explicit surface_border_base(sGlobalVars &iGlobals);

    virtual ~surface_border_base();
};

/***************************************************
 * surface base class
 */

class surface_base : public matrixcomponents_base {
public:
    explicit surface_base(sGlobalVars &iGlobals);

    virtual ~surface_base();
};

/***************************************************
 * base class for membrane functors
 */

class functor_membrane_base : public functor_cell_base {
public:
    functor_membrane_base(
            sGlobalVars &iGlobals
    );

    functor_membrane_base(
            sGlobalVars &iGlobals,
            std::string iName,
            std::string iFunctorGroupName
    );

    virtual ~functor_membrane_base();

    virtual membrane_linker_base *split(
            cell_base *iCell,
            membrane_part_base *iMembranePart,
            const Eigen::Vector3d &iPos
    );

    virtual void merge(
            cell_base *iCell
    );

    virtual double get_length(cell_base *iCell);

    virtual const double &get_radius() { return radius; };

    virtual double get_realRadius() { return radius * globals.settings->referenceLength; };

    virtual void update_positions(cell_base *iCell);

    virtual void make_timeStep(double &dT, cell_base *iCell);

protected:
    double &radius;
};

/***************************************************
 * baseclass for filament functors
 */

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

/***************************************************
 * base class for linker functors
 */

class functor_linker_base : public functor_cell_base {
public:
    functor_linker_base(
            sGlobalVars &iGlobals,
            std::string iName,
            std::string iFunctorGroupName
    );

    virtual ~functor_linker_base();

    virtual linker_base *create_linker(cell_base *iCell, const std::set<cellcomponents_base *> &iConnectedComponents);

    virtual void delete_linker(cell_base *iCell, linker_base *iLinker);
};

#endif
