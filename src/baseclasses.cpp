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

/*
template<class T> T grid_cell::determine_shape(unsigned iID, components_base* iComponent) {

	// get data
	std::vector<ofVec2d>& positions = iComponent->get_positions();
	std::vector<double>& parameters = iComponent->get_parameters();

	// get shapes
	switch (iComponent->get_visualObj()->get_type()) {
		case 1: {
			if (iID == 0) {
				boost::geometry::clear(lineA);
				boost::geometry::append(lineA,point_t(positions[0].x,positions[0].y));
				boost::geometry::append(lineA,point_t(positions[1].x,positions[1].y));
				return lineA;
			} else if (iID == 1) {
				boost::geometry::clear(lineB);
				boost::geometry::append(lineB,point_t(positions[0].x,positions[0].y));
				boost::geometry::append(lineB,point_t(positions[1].x,positions[1].y));
				return lineB;
			}
		}
		break;
		case 2: {
			if (iID == 0) {
				boost::geometry::clear(ringA);
				double angle = 360/__GRID_CIRCLE_SEGMENTS;
				for(unsigned long long i = 0; i < __GRID_CIRCLE_SEGMENTS; i++) {
					double x = positions[0].x + parameters[0] * cos(angle * i);
					double y = positions[0].y + parameters[0] * sin(angle * i);
					boost::geometry::append(ringA,point_t(x,y));
				}
				return ringA;
			} else if (iID == 1) {
				boost::geometry::clear(ringB);
				double angle = 360/__GRID_CIRCLE_SEGMENTS;
				for(unsigned long long i = 0; i < __GRID_CIRCLE_SEGMENTS; i++) {
					double x = positions[0].x + parameters[0] * cos(angle * i);
					double y = positions[0].y + parameters[0] * sin(angle * i);
					boost::geometry::append(ringB,point_t(x,y));
				}
				return ringB;
			}
		}
		break;
		case 3: {
			if (iID == 0) {
				boost::geometry::clear(boxA);
				boost::geometry::append(boxA,point_t(positions[0].x,positions[0].y));
				boost::geometry::append(boxA,point_t(positions[3].x,positions[3].y));
				return(boxA);
			} else if (iID == 1) {
				boost::geometry::clear(boxB);
				boost::geometry::append(boxB,point_t(positions[0].x,positions[0].y));
				boost::geometry::append(boxB,point_t(positions[3].x,positions[3].y));
				return(boxB);
			}
		}
		break;
	}
}
*/

std::pair<components_base*,ofVec2d> grid_cell::obtain_intersectingCircleLine(components_base* iRef, components_base* iCom) {
	std::vector<ofVec2d>& posRef = iRef->get_positions();
	std::vector<ofVec2d>& posCom = iCom->get_positions();
	std::vector<double>& parCom = iRef->get_parameters();
	for (unsigned long long i = 0; i < posRef.size() - 1; i++) {
		ofVec2d vecA = posRef[i+1] - posRef[i];
		ofVec2d vecB = posCom[0] - posRef[i];
		double angle = vecA.angleRad(vecB);
		if (angle < )
		if (vecB.length() * sin(angle))
	}
}

std::pair<components_base*,ofVec2d> grid_cell::obtain_intersectingLineLine(components_base* iRef, components_base* iCom) {

}

std::vector<std::pair<components_base*,ofVec2d>> grid_cell::obtain_intersecting(components_base* iComponent) {
	std::vector<std::pair<components_base*,ofVec2d>> intersecting;

	// get data of reference object
	unsigned& typeRef = iComponent->get_visualObj()->get_type();
	std::vector<ofVec2d>& posRef = iComponent->get_positions();
	std::vector<double>& parRef = iComponent->get_parameters();

	for (auto& it : components) {
		if (it != iComponent) {
			// get data of neighbor object
			unsigned& typeCom = it->get_visualObj()->get_type();
			std::vector<ofVec2d>& posCom = it->get_positions();
			std::vector<double>& parCom = it->get_parameters();

			// handle two circles
			if (typeRef == typeCom && typeRef == 2) {
				// check whether the two circles overlap
				if (posRef[0].distance(posCom[0]) < parRef[0] + parCom[0]) {
					ofVec2d v = posCom[0] - posRef[0];
					v.normalize();
					intersecting.push_back(std::make_pair(it,v));
				}
			} else
			// handle circle and line like
			if ((typeRef == 1 || typeRef == 3 || typeRef == 4) && typeCom == 2) {
				intersecting.push_back(obtain_intersectingCircleLine(iComponent,it));
			} else
			if ((typeCom == 1 || typeCom == 3 || typeCom == 4) && typeRef == 2) {
				intersecting.push_back(obtain_intersectingCircleLine(it,iComponent));
			} else
			// handle multiline objects
			if (
					(typeRef == 1 || typeRef == 3 || typeRef == 4)
					&& (typeCom == 1 || typeCom == 3 || typeCom == 4)
			) {
				intersecting.push_back(obtain_intersectingLineLine(iComponent,it));
			}
		}
	}
	return intersecting;
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
					assignedCells.push_back(&(cells
							[floor(xMin / cellLength)]
							[floor((m * xMin + f0) / cellLength)]));
					xMin += cellLength;
				}
			} else {
				double yMin = std::min(posA.y,posB.y);
				double yMax = std::max(posA.y,posB.y);
				unsigned long long x = floor(posA.x / cellLength);
				while (yMin < yMax) {
					assignedCells.push_back(&cells[x][floor(yMin / cellLength)]);
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
					assignedCells.push_back(&cells[x][yA]);
					assignedCells.push_back(&cells[x][yB]);
				} else {
					assignedCells.push_back(&cells[x][yA]);
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
				assignedCells.push_back(&cells[x][yA]);
				assignedCells.push_back(&cells[x][yB]);
				xMin += cellLength;
			}
			while (yMin < yMax) {
				unsigned long long y = floor(yMin / cellLength);
				assignedCells.push_back(&cells[xA][y]);
				assignedCells.push_back(&cells[xB][y]);
				yMin += cellLength;
			}
		}
		break;
		case 4: {

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
void components_base::set_gridCells(std::vector<grid_cell*> iGridCells) {
	gridCells = iGridCells;
}
void components_base::set_canMove(bool iCanMove) {
	canMove = iCanMove;
}
void components_base::set_canColide(bool iCanColide) {
	canColide = iCanColide;
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
 * simple surface
 ***************************/

simple_surface::simple_surface(grid_base* iGrid, double iSideLength): matrix_base(iGrid) {
	sideLength = iSideLength;
	positions.clear();
	positions.push_back(ofVec2d(0,0));
	positions.push_back(ofVec2d(0,iSideLength));
	positions.push_back(ofVec2d(iSideLength,0));
	positions.push_back(ofVec2d(iSideLength,iSideLength));
	associatedVisualObj = new visual_rectangle(this);
	associatedVisualObj->set_fillColor(1.0,1.0,1.0);
	associatedVisualObj->set_color(1.0,1.0,1.0);
}
simple_surface::~simple_surface(){
	for (unsigned long long i = 0; i < facs.size(); i++) {
		delete facs[i];
	}
}
void simple_surface::obtain_visualObjs(std::vector<visual_base*>& oVisualComponents) {
	oVisualComponents.push_back(associatedVisualObj);
	for(unsigned long long i = 0; i < facs.size(); i++) {
		facs[i]->obtain_visualObjs(oVisualComponents);
	}
}
void simple_surface::create_facs(unsigned iType, unsigned long long iCount, double iRadius) {
	boost::random::uniform_real_distribution <double> rndPosX(positions[0].x + iRadius, positions[3].x - iRadius);
	boost::random::uniform_real_distribution <double> rndPosY(positions[0].y + iRadius, positions[3].y - iRadius);
	switch(iType) {
		case 0: {
			for(unsigned long long i = 0; i < iCount; i++) {
				facs.push_back(new fac(grid,iRadius,rndPosX(RandomGen),rndPosY(RandomGen)));
			}
		}
		break;
	}
}
