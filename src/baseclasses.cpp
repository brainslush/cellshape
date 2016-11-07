#include "baseclasses.h"

boost::random::mt19937 RandomGen;

/***************************
 * random_base
 ***************************/
random_base::random_base(
		boost::random::mt19937* iGen,
		std::string iType,
		long long iA,
		long long iB)
{
	gen = iGen;
	type = 0;
	if (iType == "uniform_smallint") {
		type = 10;
		dist = new boost::random::uniform_smallint<> (iA,iB);
	}
}
random_base::random_base(
		boost::random::mt19937* iGen,
		std::string iType,
		double iA,
		double iB)
{
	gen = iGen;
	type = 0;
	if (iType == "uniform_real_distribution") {
		type = 20;
		dist = new boost::random::uniform_real_distribution<> (iA,iB);
	}
	if (iType == "normal_distribution") {
		type = 21;
		dist = new boost::random::normal_distribution<> (iA,iB);
	}
	if (iType == "lognormal_distribution") {
		type = 22;
		dist = new boost::random::lognormal_distribution<> (iA,iB);
	}
}
random_base::random_base(
		boost::random::mt19937* iGen,
		std::string iType,
		double iA)
{
	gen = iGen;
	type = 0;
	if (iType == "bernoulli_distribution") {
		type = 30;
		dist = new boost::random::bernoulli_distribution<> (iA);
	}
	if (iType == "exponential_distribution") {
		type = 31;
		dist = new boost::random::exponential_distribution<> (iA);
	}
}
random_base::random_base(
		boost::random::mt19937* iGen,
		std::string iType)
{
	gen = iGen;
	type = 0;
	if (iType == "uniform_01") {
		type = 40;
		dist = new boost::random::uniform_01<>();
	}
}

template<class T> T random_base::draw() {
	switch(type) {
		case 10:
			return (*boost::get<boost::random::uniform_smallint<>*>(dist))(*gen);
		break;
		case 20:
			return (*boost::get<boost::random::uniform_real_distribution<>*>(dist))(*gen);
		break;
		case 21:
			return (*boost::get<boost::random::normal_distribution<>*>(dist))(*gen);
		break;
		case 22:
			return (*boost::get<boost::random::lognormal_distribution<>*>(dist))(*gen);
		break;
		case 30:
			return (*boost::get<boost::random::bernoulli_distribution<>*>(dist))(*gen);
		break;
		case 31:
			return (*boost::get<boost::random::exponential_distribution<>*>(dist))(*gen);
		break;
		case 32:
			return (*boost::get<boost::random::uniform_01<>*>(dist))(*gen);
		break;
	}
}

/***************************
 * random_container
 ***************************/

random_container::random_container() {
	seed = gen.default_seed;
}
random_container::~random_container() {
	for (auto& it : distributions) {
		delete it;
	}
}
unsigned long long& random_container::get_seed() {
	return seed;
}
void random_container::set_seed() {

}
void random_container::set_seed(unsigned long long iSeed) {
	seed = iSeed;
	gen.seed(seed);
}
template<typename... A> random_base* random_container::register_random(
		std::string iType,
		A... args
) {
	random_base* temp = new random_base(&gen,iType,args...);
	distributions.insert(temp);
	return temp;
}
void random_container::unregister_random(random_base* iDist) {
	delete iDist;
	distributions.erase(iDist);
}

/***************************
 * grid_cell
 ***************************/

grid_cell::grid_cell() {

}
grid_cell::~grid_cell() {

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

std::pair<components_base*,ofVec2d> grid_cell::obtain_intersectingLineLine(components_base* iRef, components_base* iCom) {
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
		return std::make_pair(iCom,diffLB);
	}
	return std::make_pair(iRef,l1S);
}

std::vector<std::pair<components_base*,ofVec2d>> grid_cell::obtain_intersecting(
		components_base* iComponentA,
		components_base* iComponentB
) {
	std::vector<std::pair<components_base*,ofVec2d>> intersecting;
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
				intersecting.push_back(std::make_pair(iComponentB,v));
			}
		} else
		// handle circle and line like
		if ((typeA == 1 || typeA == 3 || typeA == 4) && typeB == 2) {
			temp = obtain_intersectingCircleLine(iComponentA,iComponentB);
			if(temp.first != iComponentA) {
				intersecting.push_back(temp);
			}
		} else
		if ((typeB == 1 || typeB == 3 || typeB == 4) && typeA == 2) {
			temp = obtain_intersectingCircleLine(iComponentB,iComponentA);
			if(temp.first == iComponentA) {
				temp.first = iComponentB;
				intersecting.push_back(temp);
			}
		} else
		// handle multi-line objects
		if (
				(typeA == 1 || typeA == 3 || typeA == 4)
				&& (typeB == 1 || typeB == 3 || typeB == 4)
		) {
			intersecting.push_back(obtain_intersectingLineLine(iComponentA,iComponentB));
		}
	}
	return intersecting;
}

void grid_cell::update_intersecting() {
	for (auto& it : components) {
		for (auto& check : components) {
			if (!check->get_intersectingChecked(this)) {

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
	for(unsigned long long i = 0; i < resolution; i++) {
		std::vector<grid_cell> temp;
		for(unsigned long long j = 0; j < resolution; j++) {
			temp.push_back(grid_cell());
		}
		cells.push_back(temp);
	}
}
grid_base::~grid_base(){

}
void grid_base::register_component(components_base* iComponent) {
	components.insert(iComponent);
	update_component(iComponent);
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
	std::vector<std::pair<grid_cell*,bool>> assignedCells;
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
					assignedCells.push_back(std::make_pair(
							&(cells
							[floor(xMin / cellLength)]
							[floor((m * xMin + f0) / cellLength)]),
							false
					));
					xMin += cellLength;
				}
			} else {
				double yMin = std::min(posA.y,posB.y);
				double yMax = std::max(posA.y,posB.y);
				unsigned long long x = floor(posA.x / cellLength);
				while (yMin < yMax) {
					assignedCells.push_back(std::make_pair(
							&cells[x][floor(yMin / cellLength)],
							false
					));
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
					assignedCells.push_back(std::make_pair(
							&cells[x][yA],
							false
					));
					assignedCells.push_back(std::make_pair(
							&cells[x][yB],
							false
					));
				} else {
					assignedCells.push_back(std::make_pair(
							&cells[x][yA],
							false
					));
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
				assignedCells.push_back(std::make_pair(
						&cells[x][yA],
						false
				));
				assignedCells.push_back(std::make_pair(
						&cells[x][yB],
						false
				));
				xMin += cellLength;
			}
			while (yMin < yMax) {
				unsigned long long y = floor(yMin / cellLength);
				assignedCells.push_back(std::make_pair(
						&cells[xA][y],false
				));
				assignedCells.push_back(std::make_pair(
						&cells[xB][y],
						false
				));
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
 * visual_base
 ***************************/

visual_base::visual_base(unsigned iType, components_base* iComponent) {
	type = iType;
	color.r = 0.0;
	color.g = 0.0;
	color.b = 0.0;
	color.a = 0.0;
	fillColor.r = 0.0;
	fillColor.g = 0.0;
	fillColor.b = 0.0;
	fillColor.a = 0.0;
	associatedComponent = iComponent;
}
visual_base::~visual_base(){

}
components_base& visual_base::get_associatedComponent() {
	return *associatedComponent;
}
ofFloatColor& visual_base::get_color() {
	return color;
}
ofFloatColor& visual_base::get_fillColor() {
	return fillColor;
}
unsigned& visual_base::get_type() {
	return type;
}
std::vector<ofVec2d>& visual_base::get_positions() {
	return associatedComponent->get_positions();
}
std::vector<double>& visual_base::get_parameters() {
	return associatedComponent->get_parameters();
}
void visual_base::set_associatedComponent(components_base* iComponent) {
	associatedComponent = iComponent;
}
void visual_base::set_color(double iRed, double iGreen, double iBlue, double iAlpha) {
	color.r = iRed;
	color.g = iGreen;
	color.b = iBlue;
	color.a = iAlpha;
}
void visual_base::set_color(double iRed, double iGreen, double iBlue) {
	color.r = iRed;
	color.g = iGreen;
	color.b = iBlue;
	color.a = 1.0;
}
void visual_base::set_fillColor(double iRed, double iGreen, double iBlue, double iAlpha) {
	fillColor.r = iRed;
	fillColor.g = iGreen;
	fillColor.b = iBlue;
	fillColor.a = iAlpha;
}
void visual_base::set_fillColor(double iRed, double iGreen, double iBlue) {
	fillColor.r = iRed;
	fillColor.g = iGreen;
	fillColor.b = iBlue;
	fillColor.a = 1.0;
}

/***************************
 * visual_line
 ***************************/
visual_line::visual_line(components_base* iComponent) : visual_base(1,iComponent) {

}
visual_line::~visual_line() {

}

/***************************
 * visual_ellipse
 ***************************/
visual_ellipse::visual_ellipse(components_base* iComponent) : visual_base(2,iComponent) {

}
visual_ellipse::~visual_ellipse() {

}

/***************************
 * visual_rectangle
 ***************************/
visual_rectangle::visual_rectangle(components_base* iComponent) : visual_base(3,iComponent) {

}
visual_rectangle::~visual_rectangle() {

}

/***************************
 * visual_triangle
 ***************************/
visual_triangle::visual_triangle(components_base* iComponent) : visual_base(4,iComponent) {

}
visual_triangle::~visual_triangle() {

}

/***************************
 * Components Base
 ***************************/
components_base::components_base(grid_base* iGrid){
	canMove = true;
	canColide = true;
	ignoreColide.insert(this);
	associatedVisualObj = NULL;
	timeStamp = ofGetFrameNum();
	grid = iGrid;
	grid->register_component(this);
}
components_base::~components_base() {
	grid->unregister_component(this);
	if (associatedVisualObj != NULL) {
		delete associatedVisualObj;
	}
}
bool& components_base::get_canMove() {
	return canMove;
}
bool& components_base::get_canColide() {
	return canColide;
}
std::vector<ofVec2d>& components_base::get_positions() {
	return positions;
}
std::vector<double>& components_base::get_parameters() {
	return parameters;
}
unsigned long long& components_base::get_timeStamp() {
	return timeStamp;
}
std::vector<grid_cell*>& components_base::get_gridCells() {
	return gridCells;
}
visual_base* components_base::get_visualObj() {
	return associatedVisualObj;
}
bool components_base::get_intersectingChecked(grid_cell* iGridCell) {
	for (auto& it : gridCells) {
		if (it.first == iGridCell) {
			return it.second;
		}
	}
	return false;
}
void components_base::set_gridCells(std::vector<std::pair<grid_cell*,bool>> iGridCells) {
	gridCells = iGridCells;
}
void components_base::set_canMove(bool iCanMove) {
	canMove = iCanMove;
}
void components_base::set_canColide(bool iCanColide) {
	canColide = iCanColide;
}
void components_base::set_intersectingChecked(grid_cell* iGridCell) {
	for (auto& it : gridCells) {
		if (it.first == iGridCell) {
			it.second = true;
		}
	}
}
void components_base::obtain_visualObjs(std::vector<visual_base*>& iVisualObjs) {
	/*do nothing*/
}
void components_base::add_ignoreColide(components_base* iIgnore) {
	ignoreColide.insert(iIgnore);
}
void components_base::make_timeStep(
		double iTime,
		unsigned long long iTimeStamp
) {
	/* do nothing */
}
void components_base::update_timeStamp() {
	timeStamp = ofGetFrameNum();
};

/***************************
 * Cell Base
 ***************************/

cell_base::cell_base(grid_base* iGrid): components_base(iGrid) {
	canMove = true;
}
cell_base::~cell_base(){

}

/***************************
 * Matrix Base
 ***************************/

matrix_base::matrix_base(grid_base* iGrid): components_base(iGrid) {
	canColide = false;
	canMove = false;
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
	double dAngle = 360 / (double)iResolution;
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
 * cell base
 ***************************/

cell::cell(
		grid_base* iGrid,
		double iX,
		double iY,
		unsigned long long iResolution
): cell_base(iGrid) {
	membrane.insert(new membrane_base(iGrid,iX,iY,200,iResolution));
	maxFillamentLength = 1;
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
	//grid = iGrid;
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
