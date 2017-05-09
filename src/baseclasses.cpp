#include "baseclasses.h"

boost::random::mt19937 RandomGen;

/***************************
 * Components Base
 ***************************/

components_base::components_base(globalVars& iGlobals): base(), globals(iGlobals){
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
void components_base::make_timeStep() {
	/* do nothing */
}

/***************************
 * Cell Base
 ***************************/

cell_base::cell_base(globalVars& iGlobals): components_base(iGlobals) {
	canMove = true;
	typeID += 10000;
}
cell_base::~cell_base(){}
void cell_base::destory_fillament(fillament_base * iFillament){}

/***************************
 * Matrix Base
 ***************************/

matrix_base::matrix_base(globalVars& iGlobals): components_base(iGlobals) {
	canColide = false;
	canMove = false;
	typeID += 20000;
}
matrix_base::~matrix_base(){

}

/***************************
 * crosslinker base
 ***************************/

crosslinker_base::crosslinker_base(
	globalVars& iGlobals,
	cell_base& iCell
):
	cellcomponents_base(iGlobals,iCell)
{
	canColide = false;
	canMove = true;
	force = ofVec2d(0,0);
	typeID += 100;
	iGlobals.grid->register_component(this);
};
crosslinker_base::~crosslinker_base() {

};
std::set<fillament_base*>& crosslinker_base::get_connectedFillaments() {
	return connectedFillaments;
}
ofVec2d& crosslinker_base::get_force(unsigned long long iTimeStamp) {
	if (timeStamp != iTimeStamp) {
		return force;
	} else {
		return force;
	}
}
void crosslinker_base::add_connectedFillament(fillament_base* iFillament) {
	connectedFillaments.insert(iFillament);
};
void crosslinker_base::remove_connectedFillament(fillament_base* iFillament) {
	connectedFillaments.erase(iFillament);
};

/***************************
 * fillament base
 ***************************/

fillament_base::fillament_base(
	globalVars& iGlobals,
	cell_base& iCell
):
	cellcomponents_base(iGlobals,iCell)
{
	canColide = true;
	canMove = true;
	associatedVisualObj = new visual_line(this);
	typeID += 200;
	iGlobals.grid->register_component(this);
}
fillament_base::~fillament_base(){
	delete associatedVisualObj;
}
void fillament_base::set_positions(double iX1, double iY1, double iX2, double iY2){
	positions[0].x = iX1;
	positions[0].y = iY1;
	positions[1].x = iX2;
	positions[1].y = iY2;
}
void fillament_base::add_connectedCrosslinker(crosslinker_base* iCrosslinker){
	connectedCrosslinkers.insert(iCrosslinker);
}
void fillament_base::remove_connectedCrosslinker(crosslinker_base* iCrosslinker){
	connectedCrosslinkers.erase(iCrosslinker);
}
void fillament_base::make_timeStep(double &iTime){

};

/***************************
 * actin
 ***************************/

actin::actin(
	globalVars& iGlobals,
	cell_base& iCell,
	ofVec2d iStart,
	ofVec2d iTmVelocity,
	double iMaxLength,
	double iLifeTime,
	double iStallingForce
):
	fillament_base(iGlobals,iCell),
	birthTime(iGlobals.time),
	maxLength(iMaxLength),
	lifeTime(iLifeTime),
	stallingForce(iStallingForce)
{
	positions.clear();
	positions.push_back(iStart);
	positions.push_back(iStart);
	tmVelocity = iTmVelocity;
	tail = NULL;
	typeID += 1;
}
actin::~actin() {
	if(tail != NULL) {
		delete tail;
	}
}
void actin::update_force() {
	if (!force.isUpdated()) {

		if (tail != NULL) {
			force += tail->get_force();
		}
		/*
		for (auto& it : connectedCrosslinkers) {
			force += it->get_force();
		}
		*/
		force.set_updated(true);
	}
}
variable_type<ofVec2d>& actin::get_force() {
	update_force();
	return force;
}
void actin::make_timeStep() {
	// destroy if it exceeds life time
	if (birthTime + lifeTime < globals.time) {
		cell.destory_fillament(this);
	} else {

	}
}

/***************************
 * volume base
 ***************************/

volume_base::volume_base(
	globalVars& iGlobals,
	cell_base& iCell
):
	cellcomponents_base(iGlobals,iCell)
{
	typeID += 300;
};
volume_base::~volume_base() {

}

/***************************
 * membrane part
 ***************************/

membrane_part::membrane_part(
	globalVars& iGlobals,
	cell_base& iCell,
	double iX1,double iY1,
	double iX2,double iY2
): cellcomponents_base(iGlobals,iCell) {
	positions.clear();
	positions.push_back(ofVec2d(iX1,iY1));
	positions.push_back(ofVec2d(iX2,iY2));
	neighbours.push_back(this);
	neighbours.push_back(this);
	associatedVisualObj = new visual_line(this);
	associatedVisualObj->set_color(0.0,0.0,0.0);
	associatedVisualObj->set_fillColor(0.0,0.0,0.0);
	typeID += 400;
	iGlobals.grid->register_component(this);
};
membrane_part::~membrane_part(){

};
void membrane_part::obtain_visualObjs(std::vector<visual_base*>& iVisualObjs) {
	iVisualObjs.push_back(associatedVisualObj);
}
void membrane_part::set_neighbours(membrane_part& iPartA,membrane_part& iPartB){
	neighbours[0] = &iPartA;
	neighbours[1] = &iPartB;
};
void membrane_part::make_timeStep(double iTime, unsigned long long iTimeStamp){

};


/***************************
 * membrane base
 ***************************/

// circular membrane
membrane_base::membrane_base(
	globalVars& iGlobals,
	cell_base& iCell,
	double iX,
	double iY,
	double iRadius,
	unsigned long long iResolution
): cellcomponents_base(iGlobals,iCell) {
	double dAngle = 2*PI/(double)iResolution;
	for (unsigned long long i = 0; i < iResolution; i++) {
		parts.push_back(new membrane_part(
			iGlobals,
			iCell,
			iRadius * cos(i    *dAngle) + iX,
			iRadius * sin(i    *dAngle) + iY,
			iRadius * cos((i+1)*dAngle) + iX,
			iRadius * sin((i+1)*dAngle) + iY
		));
	};
	for (unsigned long long i = 0; i < iResolution; i++) {
		parts[i]->set_neighbours(*parts[(i-1)%iResolution],*parts[(i+1)%iResolution]);
	};
	area.set_updated(false);
	length.set_updated(false);
	update_area();
	update_length();
	typeID += 500;
};
membrane_base::~membrane_base() {
	for (unsigned long long i = 0; i < parts.size(); i++) {
		delete parts[i];
	}
}

/* calculate area */
void membrane_base::update_area() {
	if(!area.isUpdated()) {
		double temp = 0;
		/* calculate 2D volume aka the area */
		for(auto& it : parts) {
			ofVec2d& posA = it->get_positions()[0];
			ofVec2d& posB = it->get_positions()[1];
			temp += -1 * posB.x * posA.y + posA.x * posB.y;
		};
		area = temp;
		area.set_updated(true);
	}
}
/* calculate length */
void membrane_base::update_length() {
	if (!length.isUpdated()) {
		double temp = 0;
		for (auto& it : parts) {
			ofVec2d& posA = it->get_positions()[0];
			ofVec2d& posB = it->get_positions()[1];
			temp += posA.distance(posB);
		}
		length = temp;
		length.set_updated(true);
	}
} 

/* */
void membrane_base::obtain_visualObjs(std::vector<visual_base*>& oVisualComponents) {
	for(unsigned long long i = 0; i < parts.size(); i++) {
		parts[i]->obtain_visualObjs(oVisualComponents);
	}
}

/* get volume */
variable_type<double>& membrane_base::get_area() {
	update_area();
	return area;
}
variable_type<double>& membrane_base::get_length() {
	update_length();
	return length;
}
/* update feature of the membrane */
void membrane_base::make_timeStep(double iTime, unsigned long long iTimeStamp) {
	
}

/***************************
 * cell
 ***************************/

cell::cell(
	globalVars& iGlobals,
	double iX,
	double iY,
	unsigned long long iResolution
): cell_base(iGlobals) {
	membrane.insert(new membrane_base(iGlobals,*this,iX,iY,200,iResolution));
	maxFillamentLength = 1;
	typeID += 600;
}
cell::~cell(){
	for (auto& it : membrane) {
		delete it;
	}
	for (auto& it : fillaments) {
		delete it;
	}
	for (auto& it : volumes) {
		delete it;
	}
}
void cell::obtain_visualObjs(std::vector<visual_base*>& oVisualComponents) {
	for(auto& it : membrane) {
		it->obtain_visualObjs(oVisualComponents);
	}
	for(auto& it : fillaments) {
		it->obtain_visualObjs(oVisualComponents);
	}
	for(auto& it : volumes) {
		it->obtain_visualObjs(oVisualComponents);
	}
}
void cell::create_fillament() {
	/* Needs to be enhanced */

	if (fillaments.size() == 0) {

	    fillaments.insert(new actin(globals,*this,ofVec2d(0,0),ofVec2d(0.01,0),100,2000,10));
	}
}
void cell::destory_fillament(fillament_base * iFillament) {
	fillaments.erase(iFillament);
	delete iFillament;
}
void cell::make_timeStep() {
	
}

/***************************
 * FAC base
 ***************************/

fac::fac(
	globalVars& iGlobals,
	double iRadius,
	double iX,
	double iY
): matrix_base(iGlobals) {
	parameters.clear();
	parameters.push_back(iRadius);
	parameters.push_back(iRadius);
	positions.clear();
	positions.push_back(ofVec2d(iX,iY));
	associatedVisualObj = new visual_ellipse(this);
	associatedVisualObj->set_color(1.0,0.0,0.0);
	associatedVisualObj->set_fillColor(1.0,0.0,0.0);
	typeID += 100;
	iGlobals.grid->register_component(this);
}
fac::~fac() {
}
void fac::obtain_visualObjs(std::vector<visual_base*>& oVisualComponents) {
	oVisualComponents.push_back(associatedVisualObj);
}
void fac::set_radius(double iRadius) {
	parameters.clear();
	parameters.push_back(iRadius);
	parameters.push_back(iRadius);
}
void fac::set_position(double iX, double iY) {
	positions.clear();
	positions.push_back(ofVec2d(iX,iY));
}

/***************************
 * surface_borders
 ***************************/

surface_border::surface_border(
	globalVars& iGlobals,
	simple_surface* iSurface,
	ofVec2d iStart,
	ofVec2d iEnd
): matrix_base(iGlobals) {
	surface = iSurface;
	positions.clear();
	positions.push_back(iStart);
	positions.push_back(iEnd);
	associatedVisualObj = new visual_line(this);
	associatedVisualObj->set_fillColor(1.0,1.0,1.0);
	associatedVisualObj->set_color(1.0,1.0,1.0);
	typeID += 200;
	iGlobals.grid->register_component(this);
}
surface_border::~surface_border() {
	delete associatedVisualObj;
}
void surface_border::obtain_visualObjs(std::vector<visual_base*>& oVisualComponents) {
	oVisualComponents.push_back(associatedVisualObj);
}

/***************************
 * simple surface
 ***************************/

simple_surface::simple_surface(
	globalVars& iGlobals,
	double iSideLength
): matrix_base(iGlobals) {
	sideLength = iSideLength;
	positions.clear();
	positions.push_back(ofVec2d(0,0));
	positions.push_back(ofVec2d(0,iSideLength));
	positions.push_back(ofVec2d(iSideLength,iSideLength));
	positions.push_back(ofVec2d(iSideLength,0));
	borders.push_back(new surface_border(iGlobals,this,positions[0],positions[1]));
	borders.push_back(new surface_border(iGlobals,this,positions[1],positions[2]));
	borders.push_back(new surface_border(iGlobals,this,positions[2],positions[3]));
	borders.push_back(new surface_border(iGlobals,this,positions[3],positions[0]));
	typeID += 300;
}
simple_surface::~simple_surface(){
	for (auto& it : borders) {
		delete it;
	}
	for (auto& it : facs) {
		delete it;
	}
}
void simple_surface::obtain_visualObjs(std::vector<visual_base*>& oVisualComponents) {
	for (auto& it : borders) {
		it->obtain_visualObjs(oVisualComponents);
	}
	for (auto& it : facs) {
		it->obtain_visualObjs(oVisualComponents);
	}
}
void simple_surface::create_facs(unsigned iType, unsigned long long iCount, double iRadius) {
	boost::random::uniform_real_distribution <double> rndPosX(positions[0].x + iRadius, positions[2].x - iRadius);
	boost::random::uniform_real_distribution <double> rndPosY(positions[0].y + iRadius, positions[2].y - iRadius);
	switch(iType) {
		case 0: {
			for (unsigned long long i = 0; i < iCount; i++) {
				facs.push_back(new fac(globals,iRadius,rndPosX(RandomGen),rndPosY(RandomGen)));
			}
		}
		break;
	}
}

cellcomponents_base::cellcomponents_base(
	globalVars& iGlobals,
	cell_base& iCell
):
	components_base(iGlobals),
	cell(iCell)
{}
cellcomponents_base::~cellcomponents_base() {}
