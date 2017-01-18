#include "baseclasses.h"

boost::random::mt19937 RandomGen;

/***************************
 * grid_borders
 ***************************/

grid_border::grid_border(
		double iX1,
		double iY1,
		double iX2,
		double iY2
	): base() {
	positions.clear();
	positions.push_back(ofVec2d(iX1,iY1));
	positions.push_back(ofVec2d(iX2,iY2));
	associatedVisualObj = new visual_line(this);
	associatedVisualObj->set_color(0,0,0,1);
	associatedVisualObj->set_fillColor(0,0,0,1);
}
grid_border::~grid_border() {
	delete associatedVisualObj;
}
void grid_border::obtain_visualObjs(std::vector<visual_base*>& iVisualObjs){
	iVisualObjs.push_back(associatedVisualObj);
}


/***************************
 * grid_cell
 ***************************/

grid_cell::grid_cell(
		double iX1,
		double iY1,
		double iX2,
		double iY2
	): base() {
	borders.clear();
	borders.push_back(new grid_border(iX1,iY1,iX1,iY2));
	borders.push_back(new grid_border(iX1,iY2,iX2,iY2));
	borders.push_back(new grid_border(iX2,iY2,iX2,iY1));
	borders.push_back(new grid_border(iX2,iY1,iX1,iY1));
	positions.clear();
	positions.push_back(ofVec2d(iX1,iY1));
	positions.push_back(ofVec2d(iX1,iY2));
	positions.push_back(ofVec2d(iX2,iY2));
	positions.push_back(ofVec2d(iX2,iY1));
	associatedVisualObj = new visual_rectangle(this);
	associatedVisualObj->set_color(0,0,1,1);
	associatedVisualObj->set_fillColor(0,0,1,1);
}
grid_cell::~grid_cell() {
	for (auto& it : borders) {
		delete it;
	}
}
std::set<components_base*>& grid_cell::get_components() {
	return components;
}

std::pair<components_base*,ofVec2d> grid_cell::obtain_intersectingCircleLine(components_base* iRef, components_base* iCom) {
	std::vector<ofVec2d>& posRef = iRef->get_positions();
	std::vector<ofVec2d>& posCom = iCom->get_positions();
	std::vector<double>& parCom = iRef->get_parameters();
	ofVec2d r;

	ofVec2d d = posRef[1] - posRef[0];
	ofVec2d f = posRef[1] - posCom[0];

	double a = d.dot(d);
	double b = 2*f.dot(d);
	double c = f.dot(f) - parCom[0]*parCom[0];
	double D = b*b - 4*a*c;

	if (D < 0) {
		// no intersection
		return std::make_pair(iRef,d);
	} else {
		D = sqrt(D);
		double t1 = (-D-b)/(2*a);
		double t2 = (D-b)/(2*a);

		if ( t1 >= 0 && t1 <= 1) {
			r = ofVec2d(posRef[0].x + t1 * d.x,posRef[0].y + t1 * d.x);
			r -= posCom[0];
			r.normalize();
			return std::make_pair(iCom,r);
		}
		if ( t2 >= 0 && t2 <= 1) {
			r = ofVec2d(posRef[0].x + t1 * d.x,posRef[0].y + t1 * d.x);
			r -= posCom[0];
			r.normalize();
			return std::make_pair(iCom,r);
		}
		// no intersection
		return std::make_pair(iRef,d);
	}
	return std::make_pair(iRef,posRef[0]);
}
std::pair<components_base*,std::pair<ofVec2d,ofVec2d>> grid_cell::obtain_intersectingLineLine(components_base* iRef, components_base* iCom) {
	std::vector<ofVec2d>& posRef = iRef->get_positions();
	std::vector<ofVec2d>& posCom = iCom->get_positions();

	ofVec2d& l1S = posRef[0];
	ofVec2d& l1E = posRef[1];
	ofVec2d& l2S = posCom[0];
	ofVec2d& l2E = posCom[1];

	double compareA, compareB;
	ofVec2d diffLA = l1E - l1S;
	ofVec2d diffLB = l2E - l2S;
	compareA = diffLA.x*l1S.y - diffLA.y*l1S.x;
	compareB = diffLB.x*l2S.y - diffLB.y*l2S.x;
	if ((
			( ( diffLA.x*l2S.y - diffLA.y*l2S.x ) < compareA ) ^
			( ( diffLA.x*l2E.y - diffLA.y*l2E.x ) < compareA )
	)&&(
			( ( diffLB.x*l1S.y - diffLB.y*l1S.x ) < compareB ) ^
			( ( diffLB.x*l1E.y - diffLB.y*l1E.x) < compareB )
	)) {
		diffLB.normalize();
		return std::make_pair(iCom,std::make_pair(diffLA,diffLB));
	}
	return std::make_pair(iRef,std::make_pair(l1S,l2S));
}

void grid_cell::obtain_visualObjs(std::vector<visual_base*>& iVisualObjs) {
	if (components.size() > 0) {
		iVisualObjs.push_back(associatedVisualObj);
	}
	for (auto& it : borders) {
		it->obtain_visualObjs(iVisualObjs);
	}
}
void grid_cell::obtain_intersecting(components_base* iComponentA,components_base* iComponentB) {
	std::pair<components_base*,ofVec2d> temp;

	// get data of reference and neighbor object object
	unsigned& typeA = iComponentA->get_visualObj()->get_type();
	unsigned& typeB = iComponentB->get_visualObj()->get_type();
	std::vector<ofVec2d>& posA = iComponentA->get_positions();
	std::vector<double>& parA = iComponentA->get_parameters();
	std::vector<ofVec2d>& posB = iComponentB->get_positions();
	std::vector<double>& parB = iComponentB->get_parameters();

	if (iComponentA != iComponentB) {

		// handle two circles
		if (typeA == typeB && typeA == 2) {
			// check whether the two circles overlap
			if (posA[0].distance(posB[0]) < parA[0] + parB[0]) {
				ofVec2d v = posB[0] - posA[0];
				v.normalize();
				iComponentA->add_intersector(iComponentB,v);
				iComponentB->add_intersector(iComponentA,-v);
			}
		} else
		// handle circle and line like
		if ((typeA == 1 || typeA == 3 || typeA == 4) && typeB == 2) {
			temp = obtain_intersectingCircleLine(iComponentA,iComponentB);
			if(temp.first != iComponentA) {
				iComponentA->add_intersector(iComponentB, temp.second);
				iComponentB->add_intersector(iComponentA,-temp.second);
			}
		} else
		if ((typeB == 1 || typeB == 3 || typeB == 4) && typeA == 2) {
			temp = obtain_intersectingCircleLine(iComponentB,iComponentA);
			if(temp.first == iComponentA) {
				iComponentA->add_intersector(iComponentB,temp.second);
				iComponentB->add_intersector(iComponentA,temp.second);
			}
		} else
		// handle multi-line objects
		if (
				(typeA == 1 || typeA == 3 || typeA == 4)
				&& (typeB == 1 || typeB == 3 || typeB == 4)
		) {
			std::pair<components_base*,std::pair<ofVec2d,ofVec2d>> temp2 = obtain_intersectingLineLine(iComponentA,iComponentB);
			if (temp2.first == iComponentA) {
				iComponentA->add_intersector(iComponentB,temp2.second.first);
				iComponentB->add_intersector(iComponentA,temp2.second.second);
			}
		}
	}
}
void grid_cell::update_intersecting() {
	std::set<unsigned>::iterator itType;
	for (auto& it : components) {
		it->clear_intersectors();
	}
	for (auto& itA : components) {
		for (auto& itB : components) {
			if (
					(itA->get_ignoreIntersect().find(itA->get_typeID()) == itA->get_ignoreIntersect().end()) &&
					(itB->get_ignoreIntersect().find(itA->get_typeID()) == itB->get_ignoreIntersect().end())
			) {
				obtain_intersecting(itA,itB);
			}
		}
	}
}
void grid_cell::remove_component(components_base* iComponent) {
	components.erase(iComponent);
}
void grid_cell::add_component(components_base* iComponent) {
	components.insert(iComponent);
}

/***************************
 * grid_base
 ***************************/

grid_base::grid_base(unsigned long long iResolution, double iSideLength) {
	sideLength = iSideLength;
	resolution = iResolution;
	double stepLength = iSideLength / (double) iResolution;
	for(unsigned long long i = 0; i < resolution; i++) {
		for(unsigned long long j = 0; j < resolution; j++) {
			double iX1 = stepLength * i;
			double iY1 = stepLength * j;
			double iX2 = stepLength * (i + 1);
			double iY2 = stepLength * (j + 1);
			cells.push_back(new grid_cell(iX1,iY1,iX2,iY2));
		}

	}
}
grid_base::~grid_base(){
	for (auto& it : cells) {
		delete it;
	}
}

void grid_base::obtain_visualObjs(std::vector<visual_base*>& iVisualObjs) {
	for (auto& it : cells) {
		it->obtain_visualObjs(iVisualObjs);
	}
}
void grid_base::register_component(components_base* iComponent) {
	if (iComponent->get_visualObj()) {
		components.insert(iComponent);
		//update_component(iComponent);
	} else {
		std::cout << "Could not register object with ID: " << iComponent->get_typeID() << "\n Visual object is missing";
	}
}
void grid_base::unregister_component(components_base* iComponent) {
	for (auto& it : iComponent->get_gridCells()) {
		it->remove_component(iComponent);
	}
	components.erase(iComponent);
}
void grid_base::update_component(components_base* iComponent) {
	double cellLength = sideLength / (double)resolution;
	// remove entries from old cells first
	for (auto& it : iComponent->get_gridCells()) {
		it->remove_component(iComponent);
	}
	//assign new cells
	std::vector<grid_cell*> assignedCells;
	switch (iComponent->get_visualObj()->get_type()) {
		case 1: {
			const ofVec2d& posA = iComponent->get_positions()[0];
			const ofVec2d& posB = iComponent->get_positions()[1];

			// get slope
			double m = std::numeric_limits<double>::infinity();
			if (posA.x != posB.x) {
				m = (posA.y - posB.y) / (posA.x - posB.x);
			}
			// get cells
			if (m < std::numeric_limits<double>::max()) {
				// get f(0)
				double f0 = posA.y - m * posA.x;
				double xMin = std::min(posA.x,posB.x);
				double xMax = std::max(posA.x,posB.x);
				while (xMin < xMax) {
					unsigned long long indexTestA = floor(xMin / cellLength);
					unsigned long long indexTestB = floor((m * xMin + f0) / cellLength);
					unsigned long long index = std::min(resolution-1,indexTestA) * resolution + std::min(resolution-1,indexTestB);
					assignedCells.push_back(cells[index]);
					cells[index]->add_component(iComponent);
					xMin += cellLength;
				}
			} else {
				double yMin = std::min(posA.y,posB.y);
				double yMax = std::max(posA.y,posB.y);
				unsigned long long x = floor(posA.x / cellLength);
				while (yMin < yMax) {
					unsigned long long indexTest = floor(yMin / cellLength);
					unsigned long long index = std::min(resolution-1,x) * resolution + std::min(resolution-1,indexTest);
					assignedCells.push_back(cells[index]);
					cells[index]->add_component(iComponent);
					yMin += cellLength;
				}
			}
		}
		break;
		case 2: {
			// get position and parameters
			const ofVec2d& pos = iComponent->get_positions()[0];
			const double& a = iComponent->get_parameters()[0];
			const double& b = iComponent->get_parameters()[1];
			// get lowest x
			double xMin = pos.x - a;
			double xMax = pos.x + a;
			// get grid cells
			while (xMin < xMax) {
				double y = sqrt(pow(b,2) * (1 - pow((xMin - pos.x) / a,2)));
				unsigned long long yA = floor((y + pos.y) / cellLength);
				unsigned long long yB = floor((-y + pos.y) / cellLength);
				unsigned long long x = floor(xMin / cellLength);
				if (y != 0) {
					unsigned long long indexA = x * resolution + yA;
					unsigned long long indexB = x * resolution + yB;
					assignedCells.push_back(cells[indexA]);
					assignedCells.push_back(cells[indexB]);
					cells[indexA]->add_component(iComponent);
					cells[indexB]->add_component(iComponent);
				} else {
					unsigned long long index = x * resolution + yA;
					assignedCells.push_back(cells[index]);
					cells[index]->add_component(iComponent);
				}
				xMin += cellLength;
			}
		}
		break;
		case 3: {
			// get position and parameters
			ofVec2d& posA = iComponent->get_positions()[0];
			ofVec2d& posB = iComponent->get_positions()[1];
			// get lowest and largest x and y
			double xMin = std::min(posA.x,posB.x);
			double xMax = std::max(posA.x,posB.x);
			double yMin = std::min(posA.y,posB.y);
			double yMax = std::max(posA.y,posB.y);
			// get grid cells
			unsigned long long xA = floor(xMin / cellLength);
			unsigned long long xB = floor(xMax / cellLength);
			unsigned long long yA = floor(yMin / cellLength);
			unsigned long long yB = floor(yMax / cellLength);
			while (xMin < xMax) {
				unsigned long long x = floor(xMin / cellLength);
				unsigned long long indexA = x * resolution + yA;
				unsigned long long indexB = x * resolution + yB;
				assignedCells.push_back(cells[indexA]);
				assignedCells.push_back(cells[indexB]);
				cells[indexA]->add_component(iComponent);
				cells[indexB]->add_component(iComponent);
				xMin += cellLength;
			}
			while (yMin < yMax) {
				unsigned long long y = floor(yMin / cellLength);
				unsigned long long indexA = xA * resolution + y;
				unsigned long long indexB = xB * resolution + y;
				assignedCells.push_back(cells[indexA]);
				assignedCells.push_back(cells[indexB]);
				yMin += cellLength;
			}
		}
		break;
	}
	iComponent->set_gridCells(assignedCells);
}
void grid_base::update_components() {
	for (auto& it : components) {
		update_component(it);
	}
}

/***************************
 * Components Base
 ***************************/
components_base::components_base(grid_base* iGrid): base(){
	canMove = true;
	canColide = true;
	grid = iGrid;
}
components_base::~components_base() {
	grid->unregister_component(this);
}
bool& components_base::get_canMove() {
	return canMove;
}
bool& components_base::get_canColide() {
	return canColide;
}
std::set<unsigned>& components_base::get_ignoreIntersect() {
	return ignoreIntersect;
}
std::set<components_base*>& components_base::get_intersectorsChecked() {
	return intersectorsChecked;
}
std::vector<grid_cell*>& components_base::get_gridCells() {
	return gridCells;
}

void components_base::set_gridCells(std::vector<grid_cell*> iGridCells) {
	gridCells = iGridCells;
}
void components_base::set_canMove(bool iCanMove) {
	canMove = iCanMove;
}
void components_base::set_canColide(bool iCanColide) {
	canColide = iCanColide;
}

void components_base::add_intersector(components_base* iIntersector, ofVec2d iIntersectorVec) {
	intersectors.push_back(iIntersector);
	intersectorsVectors.push_back(iIntersectorVec);
	intersectorsChecked.insert(iIntersector);
}
void components_base::clear_intersectors() {
	intersectors.clear();
	intersectorsVectors.clear();
	intersectorsChecked.clear();
}
void components_base::add_ignoreIntersect(unsigned iIgnore) {
	//ignoreIntersect.insert(iIgnore);
}
void components_base::make_timeStep(
		double iTime,
		unsigned long long iTimeStamp
) {
	/* do nothing */
}

/***************************
 * Cell Base
 ***************************/

cell_base::cell_base(grid_base* iGrid): components_base(iGrid) {
	canMove = true;
	typeID += 10000;
}
cell_base::~cell_base(){

}

/***************************
 * Matrix Base
 ***************************/

matrix_base::matrix_base(grid_base* iGrid): components_base(iGrid) {
	canColide = false;
	canMove = false;
	typeID += 20000;
}
matrix_base::~matrix_base(){

}

/***************************
 * crosslinker base
 ***************************/

crosslinker_base::crosslinker_base(grid_base* iGrid): cell_base(iGrid) {
	canColide = false;
	canMove = true;
	force = ofVec2d(0,0);
	typeID += 100;
	iGrid->register_component(this);
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

fillament_base::fillament_base(grid_base* iGrid): cell_base(iGrid){
	canColide = true;
	canMove = true;
	associatedVisualObj = new visual_line(this);
	typeID += 200;
	iGrid->register_component(this);
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
		grid_base* iGrid,
		ofVec2d& iStart,
		ofVec2d& iTmVelocity,
		double iMaxLength,
		double iLifeTime,
		double iStallingForce
):fillament_base(iGrid)  {
	positions.clear();
	positions.push_back(iStart);
	positions.push_back(iStart);
	tmVelocity = iTmVelocity;
	maxLength = iMaxLength;
	lifeTime = iLifeTime;
	stallingForce = iStallingForce;
	tail = NULL;
	typeID += 1;
}
actin::~actin() {
	if(tail != NULL) {
		delete tail;
	}
}
ofVec2d& actin::get_force(unsigned long long iTimeStamp) {
	if(timeStamp == iTimeStamp) {
		return force;
	} else {
		force = ofVec2d();
		if(tail != NULL){
			force += tail->get_force(iTimeStamp);
		}
		for(auto& it : connectedCrosslinkers){
			force += it->get_force(iTimeStamp);
		}
		update_timeStamp();
		return force;
	}
}
void actin::make_timeStep(double iTime, unsigned long long iTimeStamp) {
	// search and update for possible contacts or splittings

	// get forces
	ofVec2d force;
	for(auto& it : connectedCrosslinkers){
		force += it->get_force(iTimeStamp);
	}
	if(tail != NULL){
	}
}

/***************************
 * volume base
 ***************************/

volume_base::volume_base(grid_base* iGrid): cell_base(iGrid) {
	typeID += 300;
};
volume_base::~volume_base() {

}

/***************************
 * membrane part
 ***************************/

membrane_part::membrane_part(
		grid_base* iGrid,
		double iX1,double iY1,
		double iX2,double iY2
): cell_base(iGrid) {
	positions.clear();
	positions.push_back(ofVec2d(iX1,iY1));
	positions.push_back(ofVec2d(iX2,iY2));
	neighbours.push_back(this);
	neighbours.push_back(this);
	associatedVisualObj = new visual_line(this);
	associatedVisualObj->set_color(0.0,0.0,0.0);
	associatedVisualObj->set_fillColor(0.0,0.0,0.0);
	typeID += 400;
	iGrid->register_component(this);
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
		grid_base* iGrid,
		double iX,
		double iY,
		double iRadius,
		unsigned long long iResolution
): cell_base(iGrid) {
	double dAngle = 2*PI/(double)iResolution;
	for (unsigned long long i = 0; i < iResolution; i++) {
		parts.push_back(new membrane_part(
				iGrid,
				iRadius * cos(i    *dAngle) + iX,
				iRadius * sin(i    *dAngle) + iY,
				iRadius * cos((i+1)*dAngle) + iX,
				iRadius * sin((i+1)*dAngle) + iY
		));
	};
	for (unsigned long long i = 0; i < iResolution; i++) {
		parts[i]->set_neighbours(*parts[(i-1)%iResolution],*parts[(i+1)%iResolution]);
	};
	updatedVolume = false;
	initialVolume = calc_currentVolume();
	typeID += 500;
};
membrane_base::~membrane_base() {
	for (unsigned long long i = 0; i < parts.size(); i++) {
		delete parts[i];
	}
}

/* calculate volume */
double membrane_base::calc_currentVolume() {
	if(!updatedVolume) {
		double volume = 0;
		/* calculate 2D volume aka the area */
		for(auto& it : parts) {
			const ofVec2d& posA = it->get_positions()[0];
			const ofVec2d& posB = it->get_positions()[1];
			volume += -1 * posB.x * posA.y + posA.x * posB.y;
		};
		currentVolume = abs(volume * 0.5);
		updatedVolume = true;
	}
	return currentVolume;
}
/* */
void membrane_base::obtain_visualObjs(std::vector<visual_base*>& oVisualComponents) {
	for(unsigned long long i = 0; i < parts.size(); i++) {
		parts[i]->obtain_visualObjs(oVisualComponents);
	}
}

/* get volume */
double membrane_base::get_volume() {
	return calc_currentVolume();
}
/* update feature of the membrane */
void membrane_base::make_timeStep(double iTime, unsigned long long iTimeStamp) {
	updatedVolume = false;
}

/***************************
 * cell
 ***************************/

cell::cell(
		grid_base* iGrid,
		double iX,
		double iY,
		unsigned long long iResolution
): cell_base(iGrid) {
	membrane.insert(new membrane_base(iGrid,iX,iY,200,iResolution));
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
};
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
void cell::make_timeStep(double iTime, unsigned long long iTimeStamp) {

}

/***************************
 * FAC base
 ***************************/

fac::fac(
		grid_base* iGrid,
		double iRadius,
		double iX,
		double iY
): matrix_base(iGrid) {
	parameters.clear();
	parameters.push_back(iRadius);
	parameters.push_back(iRadius);
	positions.clear();
	positions.push_back(ofVec2d(iX,iY));
	associatedVisualObj = new visual_ellipse(this);
	associatedVisualObj->set_color(1.0,0.0,0.0);
	associatedVisualObj->set_fillColor(1.0,0.0,0.0);
	typeID += 100;
	iGrid->register_component(this);
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
		grid_base* iGrid,
		simple_surface* iSurface,
		ofVec2d iStart,
		ofVec2d iEnd
): matrix_base(iGrid) {
	surface = iSurface;
	positions.clear();
	positions.push_back(iStart);
	positions.push_back(iEnd);
	associatedVisualObj = new visual_line(this);
	associatedVisualObj->set_fillColor(1.0,1.0,1.0);
	associatedVisualObj->set_color(1.0,1.0,1.0);
	typeID += 200;
	iGrid->register_component(this);
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

simple_surface::simple_surface(grid_base* iGrid, double iSideLength): matrix_base(iGrid) {
	sideLength = iSideLength;
	positions.clear();
	positions.push_back(ofVec2d(0,0));
	positions.push_back(ofVec2d(0,iSideLength));
	positions.push_back(ofVec2d(iSideLength,iSideLength));
	positions.push_back(ofVec2d(iSideLength,0));
	borders.push_back(new surface_border(iGrid,this,positions[0],positions[1]));
	borders.push_back(new surface_border(iGrid,this,positions[1],positions[2]));
	borders.push_back(new surface_border(iGrid,this,positions[2],positions[3]));
	borders.push_back(new surface_border(iGrid,this,positions[3],positions[0]));
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
				facs.push_back(new fac(grid,iRadius,rndPosX(RandomGen),rndPosY(RandomGen)));
			}
		}
		break;
	}
}
