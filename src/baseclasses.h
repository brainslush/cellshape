#include "extIncludes.h"
#include "variable_type.h"
#include "base.h"
#include "random.h"
#include "grid.h"
#include "RigidBody.h"


#ifndef __H_CLASSES_BASE2
#define __H_CLASSES_BASE2

class grid_base;

struct globalVars {
    grid_base* grid;
    random_container* rndC;
    uint64_t time;
    uint64_t frameNo;
    double deltaT;
};


/*
 * Base function for all physical components.
 * It is the base call for cells and substrates
 */

class components_base : public base {
public:
    components_base(globalVars& iGlobals);
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
    std::set<variable_base*> variables;
    globalVars& globals;
};

class fillament_base;
class cell_base : public components_base {
public:
    cell_base(globalVars& iGlobals);
    virtual ~cell_base();
    virtual void destory_fillament(fillament_base* iFillament);
protected:
};

class cellcomponents_base : public components_base {
public:
    cellcomponents_base(globalVars& iGlobals,cell_base& iCell);
    virtual ~cellcomponents_base();
protected:
    cell_base& cell;
};

class matrix_base : public components_base {
public:
    matrix_base(globalVars& iGlobals);
    virtual ~matrix_base();
protected:
};


/***************************
* crosslinkers
***************************/
class crosslinker_base : public cellcomponents_base {
public:
    crosslinker_base(globalVars& iGlobals,cell_base& iCell);
    virtual ~crosslinker_base();

    virtual std::set<fillament_base*>& get_connectedFillaments();
    virtual ofVec2d& get_force(unsigned long long iTimeStamp);

    virtual void add_connectedFillament(fillament_base* iFillament);
    virtual void remove_connectedFillament(fillament_base* iFillament);
protected:
    std::set<fillament_base*> connectedFillaments;
    ofVec2d force;
};

class crosslinker_static : public crosslinker_base {
public:
    crosslinker_static(globalVars& iGlobals,cell_base& iCell);
    virtual ~crosslinker_static();
};

class crosslinker_friction : public crosslinker_base {
public:
    crosslinker_friction(globalVars& iGlobals,cell_base& iCell);
    virtual ~crosslinker_friction();
};

/***************************
* fillaments
***************************/
class fillament_base : public cellcomponents_base {
public:
    fillament_base(globalVars& iGlobals, cell_base& iCell);
    virtual ~fillament_base();
    virtual void set_positions(double iX1, double iY1, double iX2, double iY2);
    virtual void add_connectedCrosslinker(crosslinker_base* iCrosslinker);
    virtual void remove_connectedCrosslinker(crosslinker_base* iCrosslinker);
    virtual void make_timeStep(double& dT);
protected:
	std::set<crosslinker_base*> connectedCrosslinkers;
};
class functor_actin_force;
class functor_actin_torque;
class actin : public fillament_base {
public:
    actin(
        globalVars& iGlobals, cell_base& iCell,
        ofVec2d iStart,
        ofVec2d iTmVelocity,
        double iMaxLength,
        double iLifeTime,
        double iStallingForce
    );
    virtual ~actin();
    virtual void update_force();
    virtual variable_type<ofVec2d>& get_force();
    virtual void make_timeStep(double& dT);
protected:
    functor_actin_force* forceF;
    functor_actin_torque* torqueF;
    physic::RigidBody3d* rigidBody;
    variable_type<ofVec2d> tmVelocity; // treadmilling velocity
    variable_type<ofVec2d> force; // current force vector in actin element
    const uint64_t birthTime; // time when object is created
    const double maxLength; // maximum length
    const double lifeTime; // dies after lifetime
    const double stallingForce; // force at which actin doesn't treadmills anymore
    actin* tail;
};
class functor_actin_force: public physic::functor {
public:
    functor_actin_force (actin* iFillament);
    virtual ~functor_actin_force ();
    virtual Eigen::Vector3d calc (
        Eigen::Vector3d& X,
        Eigen::Vector3d& v,
        Eigen::Quaterniond& R,
        Eigen::Vector3d& L
    );
protected:
    actin* fillament;
};
class functor_actin_torque: public physic::functor {
public:
    functor_actin_torque (actin* iFillament);
    virtual ~functor_actin_torque ();
    virtual Eigen::Vector3d calc (
        Eigen::Vector3d& X,
        Eigen::Vector3d& v,
        Eigen::Quaterniond& R,
        Eigen::Vector3d& L
    );
protected:
    actin* fillament;
};
/***************************
* volume
***************************/
class volume_base : public cellcomponents_base {
public:
	volume_base(globalVars& iGlobals, cell_base& iCell);
	virtual ~volume_base();
protected:
};

/***************************
* Membrane
***************************/
class membrane_part : public cellcomponents_base {
public:
    // 2D membrane part
    membrane_part(
        globalVars& iGlobals, cell_base& iCell,
        double iX1,double iY1,
        double iX2,double iY2
    );
    virtual ~membrane_part();
    virtual void obtain_visualObjs(std::vector<visual_base*>& iVisualObjs);
    virtual void set_neighbours(membrane_part& iPartA,membrane_part& iPartB);
    virtual void make_timeStep(double& dT);
protected:
	std::vector<membrane_part*> neighbours;
};
class membrane_base : public cellcomponents_base {
public:
    membrane_base(
        globalVars& iGlobals, cell_base& iCell,
        double iX, double iY,
        double iRadius,
        unsigned long long iResolution
    );
    virtual ~membrane_base();
    virtual variable_type<double>& get_area();
    virtual variable_type<double>& get_length();
    virtual void obtain_visualObjs(std::vector<visual_base*>& oVisualComponents);
    virtual void make_timeStep(double& dT);
protected:
    variable_type<double> area = variable_type<double>("Area","A");
    variable_type<double> length = variable_type<double>("Length","l");
    const bool canColide = true;
    std::vector<membrane_part*> parts;
    virtual void update_area();
    virtual void update_length();
};
/***************************
* cell
***************************/
class cell : public cell_base {
public:
    cell(
        globalVars& iGlobals,
        double iX,
        double iY,
        unsigned long long iResolution
    );
    virtual ~cell();
    virtual void obtain_visualObjs(std::vector<visual_base*>& iVisualObjs);
    virtual void create_fillament();
    virtual void destory_fillament(fillament_base* iFillament);
    virtual void make_timeStep(double& dT);
protected:
    double maxFillamentLength;
    std::set<membrane_base*> membranes;
    std::set<fillament_base*> fillaments;
    std::set<volume_base*> volumes;
};
/***************************
* fac
***************************/
class fac:public matrix_base {
public:
    fac(
        globalVars& iGlobals,
        double iRadius,
        double iX,
        double iY
    );
    virtual ~fac();
    virtual void obtain_visualObjs(std::vector<visual_base*>& iVisualObjs);
    virtual void set_radius(double iRadius);
    virtual void set_position(double iX, double iY);
protected:
};
/***************************
* surface
***************************/
class simple_surface;
class surface_border : public matrix_base {
public:
    surface_border(
        globalVars& iGlobals,
        simple_surface* iSurface,
        ofVec2d iStart,
        ofVec2d iEnd
    );
    virtual ~surface_border();
    virtual void obtain_visualObjs(std::vector<visual_base*>& iVisualObjs);
protected:
    simple_surface* surface;
};

class simple_surface : public matrix_base {
public:
    simple_surface(
        globalVars& iGlobals,
        double iSideLength
    );
    virtual ~simple_surface();
    virtual void obtain_visualObjs(std::vector<visual_base*>& iVisualObjs);
    virtual void create_facs(unsigned iType, unsigned long long iCount, double iRadius);
protected:
    double sideLength;
    std::vector<surface_border*> borders;
    std::vector<fac*> facs;
};

#endif
