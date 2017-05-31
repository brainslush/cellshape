#include <cBaseclasses.h>

/***************************
 * Components Base
 ***************************/

components_base::components_base(sGlobalVars& iGlobals): base(), globals(iGlobals){
	canMove = true;
	canColide = true;
}
components_base::~components_base() {globals.grid->unregister_component(this);}
bool& components_base::get_canMove() {return canMove;}
bool& components_base::get_canColide() {return canColide;}

void components_base::set_canMove(bool iCanMove) {canMove = iCanMove;}
void components_base::set_canColide(bool iCanColide) {canColide = iCanColide;}
void components_base::set_componentModel(std::string, std::string) {
	/* TODO */
}
void components_base::make_timeStep(double& dT) {
	/* do nothing */
}

/***************************
 * Cell Base
 ***************************/

cell_base::cell_base(sGlobalVars& iGlobals): components_base(iGlobals) {
    canMove = true;
}
cell_base::~cell_base(){}
void cell_base::destory_filament(filament_base * iFilament){}

/***************************
 * Matrix Base
 ***************************/

matrix_base::matrix_base(sGlobalVars& iGlobals): components_base(iGlobals) {
    canColide = false;
    canMove = false;
}
matrix_base::~matrix_base(){

}

/***************************
 * crosslinker base
 ***************************/

crosslinker_base::crosslinker_base(
    sGlobalVars& iGlobals,
    cell_base& iCell
):
    cellcomponents_base(iGlobals,iCell)
{
    canColide = false;
    canMove = true;
    force = Eigen::Vector3d(0,0);
    iGlobals.grid->register_component(this);
};
crosslinker_base::~crosslinker_base() {

};
std::set<filament_base*>& crosslinker_base::get_connectedFilaments() {
    return connectedFilaments;
}
Eigen::Vector3d& crosslinker_base::get_force(unsigned long long iTimeStamp) {
    if (timeStamp != iTimeStamp) {
        return force;
    } else {
        return force;
    }
}
void crosslinker_base::add_connectedFilament(filament_base* iFilament) {
    connectedFilaments.insert(iFilament);
};
void crosslinker_base::remove_connectedFilament(filament_base* iFilament) {
    connectedFilaments.erase(iFilament);
};

/***************************
 * filament base
 ***************************/

filament_base::filament_base(
    sGlobalVars& iGlobals,
    cell_base& iCell
):
    cellcomponents_base(iGlobals,iCell)
{
    canColide = true;
    canMove = true;
    associatedVisualObj = new visual_line(this);
    iGlobals.grid->register_component(this);
    length = 0;
}
filament_base::~filament_base(){
    delete associatedVisualObj;
    associatedVisualObj = NULL;
}
void filament_base::set_positions(double iX1, double iY1, double iX2, double iY2){
    positions[0](0) = iX1;
    positions[0](1) = iY1;
    positions[1](0) = iX2;
    positions[1](1) = iY2;
}
double& filament_base::get_length() {
    length = (positions[1] - positions[0]).norm();
    return length;
}
void filament_base::add_connectedCrosslinker(crosslinker_base* iCrosslinker){
    connectedCrosslinkers.insert(iCrosslinker);
}
void filament_base::remove_connectedCrosslinker(crosslinker_base* iCrosslinker){
    connectedCrosslinkers.erase(iCrosslinker);
}
void filament_base::obtain_visualObjs(std::vector<visual_base*>& iVisualObjs) {
    iVisualObjs.push_back(associatedVisualObj);
}
void filament_base::make_timeStep(double &iTime){

};

/***************************
 * volume base
 ***************************/

volume_base::volume_base(
    sGlobalVars& iGlobals,
    cell_base& iCell
):
    cellcomponents_base(iGlobals,iCell)
{
};
volume_base::~volume_base() {

}


cellcomponents_base::cellcomponents_base(
    sGlobalVars& iGlobals,
    cell_base& iCell
):
    components_base(iGlobals),
    cell(iCell)
{

}
cellcomponents_base::~cellcomponents_base() {}
