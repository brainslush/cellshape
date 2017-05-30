#include "extIncludes.h"
//#include "variable_type.h"
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
    components_base(sGlobalVars& iGlobals);
    virtual ~components_base();
    virtual bool& get_canMove();
    virtual bool& get_canColide();
    virtual void set_canMove(bool iCanMove);
    virtual void set_canColide(bool iCanColide);
    virtual void set_componentModel(std::string, std::string);
    virtual void make_timeStep(double& dT);
protected:
    bool canMove; // is it a fixed object
    bool canColide; // can this object collide aka does it have physics?
    //std::set<variable_base*> variables;
    sGlobalVars& globals;
};

class filament_base;
class cell_base : public components_base {
public:
    cell_base(sGlobalVars& iGlobals);
    virtual ~cell_base();
    virtual void destory_filament(filament_base* iFilament);
protected:
};

class cellcomponents_base : public components_base {
public:
    cellcomponents_base(sGlobalVars& iGlobals,cell_base& iCell);
    virtual ~cellcomponents_base();
protected:
    cell_base& cell;
};

class matrix_base : public components_base {
public:
    matrix_base(sGlobalVars& iGlobals);
    virtual ~matrix_base();
protected:
};


/***************************
* crosslinkers
***************************/
class crosslinker_base : public cellcomponents_base {
public:
    crosslinker_base(sGlobalVars& iGlobals,cell_base& iCell);
    virtual ~crosslinker_base();

    virtual std::set<filament_base*>& get_connectedFilaments();
    virtual Eigen::Vector3d& get_force(unsigned long long iTimeStamp);

    virtual void add_connectedFilament(filament_base* iFilament);
    virtual void remove_connectedFilament(filament_base* iFilament);
protected:
    std::set<filament_base*> connectedFilaments;
    Eigen::Vector3d force;
};

class crosslinker_static : public crosslinker_base {
public:
    crosslinker_static(sGlobalVars& iGlobals,cell_base& iCell);
    virtual ~crosslinker_static();
};

class crosslinker_friction : public crosslinker_base {
public:
    crosslinker_friction(sGlobalVars& iGlobals,cell_base& iCell);
    virtual ~crosslinker_friction();
};

/***************************
* filaments
***************************/
class filament_base : public cellcomponents_base {
public:
    filament_base(sGlobalVars& iGlobals, cell_base& iCell);
    virtual ~filament_base();
    virtual void set_positions(double iX1, double iY1, double iX2, double iY2);
    virtual void add_connectedCrosslinker(crosslinker_base* iCrosslinker);
    virtual void remove_connectedCrosslinker(crosslinker_base* iCrosslinker);
    virtual void obtain_visualObjs(std::vector<visual_base*>& iVisualObjs);
    virtual void make_timeStep(double& dT);
protected:
	std::set<crosslinker_base*> connectedCrosslinkers;
};

/***************************
* volume
***************************/
class volume_base : public cellcomponents_base {
public:
	volume_base(sGlobalVars& iGlobals, cell_base& iCell);
	virtual ~volume_base();
protected:
};



#endif
