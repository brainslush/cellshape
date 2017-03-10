#include "grid.h"

/***************************
* grid_borders
***************************/

grid_border::grid_border(
	double iX1,
	double iY1,
	double iX2,
	double iY2
) : base() {
	positions.clear();
	positions.push_back(ofVec2d(iX1, iY1));
	positions.push_back(ofVec2d(iX2, iY2));
	associatedVisualObj = new visual_line(this);
	associatedVisualObj->set_color(0, 0, 0, 1);
	associatedVisualObj->set_fillColor(0, 0, 0, 1);
}
grid_border::~grid_border() {
	delete associatedVisualObj;
}
void grid_border::obtain_visualObjs(std::vector<visual_base*>& iVisualObjs) {
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
) : base() {
	borders.clear();
	borders.push_back(new grid_border(iX1, iY1, iX1, iY2));
	borders.push_back(new grid_border(iX1, iY2, iX2, iY2));
	borders.push_back(new grid_border(iX2, iY2, iX2, iY1));
	borders.push_back(new grid_border(iX2, iY1, iX1, iY1));
	positions.clear();
	positions.push_back(ofVec2d(iX1, iY1));
	positions.push_back(ofVec2d(iX1, iY2));
	positions.push_back(ofVec2d(iX2, iY2));
	positions.push_back(ofVec2d(iX2, iY1));
	associatedVisualObj = new visual_rectangle(this);
	associatedVisualObj->set_color(0, 0, 1, 1);
	associatedVisualObj->set_fillColor(0, 0, 1, 1);
}
grid_cell::~grid_cell() {
	for (auto& it : borders) {
		delete it;
	}
}
std::set<base*>& grid_cell::get_components() {
	return components;
}

std::pair<base*, ofVec2d> grid_cell::obtain_intersectingCircleLine(base* iRef, base* iCom) {
	std::vector<ofVec2d>& posRef = iRef->get_positions();
	std::vector<ofVec2d>& posCom = iCom->get_positions();
	std::vector<double>& parCom = iRef->get_parameters();
	ofVec2d r;

	ofVec2d d = posRef[1] - posRef[0];
	ofVec2d f = posRef[1] - posCom[0];

	double a = d.dot(d);
	double b = 2 * f.dot(d);
	double c = f.dot(f) - parCom[0] * parCom[0];
	double D = b*b - 4 * a*c;

	if (D < 0) {
		// no intersection
		return std::make_pair(iRef, d);
	}
	else {
		D = sqrt(D);
		double t1 = (-D - b) / (2 * a);
		double t2 = (D - b) / (2 * a);

		if (t1 >= 0 && t1 <= 1) {
			r = ofVec2d(posRef[0].x + t1 * d.x, posRef[0].y + t1 * d.x);
			r -= posCom[0];
			r.normalize();
			return std::make_pair(iCom, r);
		}
		if (t2 >= 0 && t2 <= 1) {
			r = ofVec2d(posRef[0].x + t1 * d.x, posRef[0].y + t1 * d.x);
			r -= posCom[0];
			r.normalize();
			return std::make_pair(iCom, r);
		}
		// no intersection
		return std::make_pair(iRef, d);
	}
	return std::make_pair(iRef, posRef[0]);
}
std::pair<base*, std::pair<ofVec2d, ofVec2d>> grid_cell::obtain_intersectingLineLine(base* iRef, base* iCom) {
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
		((diffLA.x*l2S.y - diffLA.y*l2S.x) < compareA) ^
		((diffLA.x*l2E.y - diffLA.y*l2E.x) < compareA)
		) && (
		((diffLB.x*l1S.y - diffLB.y*l1S.x) < compareB) ^
			((diffLB.x*l1E.y - diffLB.y*l1E.x) < compareB)
			)) {
		diffLB.normalize();
		return std::make_pair(iCom, std::make_pair(diffLA, diffLB));
	}
	return std::make_pair(iRef, std::make_pair(l1S, l2S));
}

void grid_cell::obtain_visualObjs(std::vector<visual_base*>& iVisualObjs) {
	if (components.size() > 0) {
		iVisualObjs.push_back(associatedVisualObj);
	}
	for (auto& it : borders) {
		it->obtain_visualObjs(iVisualObjs);
	}
}
void grid_cell::obtain_intersecting(base* iComponentA, base* iComponentB) {
	std::pair<base*, ofVec2d> temp;

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
				iComponentA->add_intersector(iComponentB, v);
				iComponentB->add_intersector(iComponentA, -v);
			}
		}
		else
			// handle circle and line like
			if ((typeA == 1 || typeA == 3 || typeA == 4) && typeB == 2) {
				temp = obtain_intersectingCircleLine(iComponentA, iComponentB);
				if (temp.first != iComponentA) {
					iComponentA->add_intersector(iComponentB, temp.second);
					iComponentB->add_intersector(iComponentA, -temp.second);
				}
			}
			else
				if ((typeB == 1 || typeB == 3 || typeB == 4) && typeA == 2) {
					temp = obtain_intersectingCircleLine(iComponentB, iComponentA);
					if (temp.first == iComponentA) {
						iComponentA->add_intersector(iComponentB, temp.second);
						iComponentB->add_intersector(iComponentA, temp.second);
					}
				}
				else
					// handle multi-line objects
					if (
						(typeA == 1 || typeA == 3 || typeA == 4)
						&& (typeB == 1 || typeB == 3 || typeB == 4)
						) {
						std::pair<base*, std::pair<ofVec2d, ofVec2d>> temp2 = obtain_intersectingLineLine(iComponentA, iComponentB);
						if (temp2.first == iComponentA) {
							iComponentA->add_intersector(iComponentB, temp2.second.first);
							iComponentB->add_intersector(iComponentA, temp2.second.second);
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
				obtain_intersecting(itA, itB);
			}
		}
	}
}
void grid_cell::remove_component(base* iComponent) {
	components.erase(iComponent);
}
void grid_cell::add_component(base* iComponent) {
	components.insert(iComponent);
}

/***************************
* grid_base
***************************/

grid_base::grid_base(unsigned long long iResolution, double iSideLength) {
	sideLength = iSideLength;
	resolution = iResolution;
	double stepLength = iSideLength / (double)iResolution;
	for (unsigned long long i = 0; i < resolution; i++) {
		for (unsigned long long j = 0; j < resolution; j++) {
			double iX1 = stepLength * i;
			double iY1 = stepLength * j;
			double iX2 = stepLength * (i + 1);
			double iY2 = stepLength * (j + 1);
			cells.push_back(new grid_cell(iX1, iY1, iX2, iY2));
		}

	}
}
grid_base::~grid_base() {
	for (auto& it : cells) {
		delete it;
	}
}

void grid_base::obtain_visualObjs(std::vector<visual_base*>& iVisualObjs) {
	for (auto& it : cells) {
		it->obtain_visualObjs(iVisualObjs);
	}
}
void grid_base::register_component(base* iComponent) {
	if (iComponent->get_visualObj()) {
		components.insert(iComponent);
		//update_component(iComponent);
	}
	else {
		std::cout << "Could not register object with ID: " << iComponent->get_typeID() << "\n Visual object is missing";
	}
}
void grid_base::unregister_component(base* iComponent) {
	for (auto& it : iComponent->get_gridCells()) {
		it->remove_component(iComponent);
	}
	components.erase(iComponent);
}
void grid_base::update_component(base* iComponent) {
	double cellLength = sideLength / (double)resolution;
	// remove entries from old cells first
	for (auto& it : iComponent->get_gridCells()) {
		it->remove_component(iComponent);
	}
	//assign new cells
	std::set<grid_cell*> assignedCells;
	switch (iComponent->get_visualObj()->get_type()) {
	case 1: {
		const ofVec2d& posA = iComponent->get_positions()[0];
		const ofVec2d& posB = iComponent->get_positions()[1];

		// get slope
		double m = std::numeric_limits<double>::infinity();
		if (posA.x != posB.x) {
			m = (posA.y - posB.y) / (posA.x - posB.x);
		}
		// get f(0)
		double f0 = (posA.y - m * posA.x);
		double f0Red = f0 / cellLength;
		// get mins and maxs
		double yMin = std::min(posA.y, posB.y);
		double yMax = std::max(posA.y, posB.y);
		double xMin = std::min(posA.x, posB.x);
		double xMax = std::max(posA.x, posB.x);
		unsigned long long indexXMin = floor(xMin / cellLength);
		unsigned long long indexXMax = floor(xMax / cellLength);
		unsigned long long indexYMin = floor(yMin / cellLength);
		unsigned long long indexYMax = floor(yMax / cellLength);
		// get cells
		if (abs(m) < 1) {
			while (indexXMin <= indexXMax) {
				// calculate y value
				unsigned long long indexTestT = floor(f0Red + m * indexXMin);
				unsigned long long indexTestB = boost::algorithm::clamp(floor(f0Red + m * (indexXMin + 1)), indexYMin, indexYMax);
				unsigned long long indexT = std::min(resolution - 1, indexXMin) * resolution + std::min(resolution - 1, indexTestT);
				unsigned long long indexB = std::min(resolution - 1, indexXMin) * resolution + std::min(resolution - 1, indexTestB);
				assignedCells.insert(cells[indexT]);
				cells[indexT]->add_component(iComponent);
				if (indexT != indexB) {
					assignedCells.insert(cells[indexB]);
					cells[indexB]->add_component(iComponent);
				}
				++indexXMin;
			}
		}
		else {
			while (indexYMin <= indexYMax) {
				if (abs(m) < std::numeric_limits<double>::max()) {
					// calculate x value
					unsigned long long indexTestL = floor((indexYMin - f0Red) / m);
					unsigned long long indexTestR = boost::algorithm::clamp(floor(((indexYMin + 1) - f0Red) / m), indexXMin, indexXMax);
					unsigned long long indexL = std::min(resolution - 1, indexTestL) * resolution + std::min(resolution - 1, indexYMin);
					unsigned long long indexR = std::min(resolution - 1, indexTestR) * resolution + std::min(resolution - 1, indexYMin);
					assignedCells.insert(cells[indexL]);
					cells[indexL]->add_component(iComponent);
					if (indexL != indexR) {
						assignedCells.insert(cells[indexR]);
						cells[indexR]->add_component(iComponent);
					}
				}
				else {
					// take care of vertical lines
					unsigned long long index = std::min(resolution - 1, indexXMin) * resolution + std::min(resolution - 1, indexYMin);
					assignedCells.insert(cells[index]);
					cells[index]->add_component(iComponent);
				}
				++indexYMin;
			}
		}
	}
			break;
	case 2: {
		// get position and parameters
		const ofVec2d& pos = iComponent->get_positions()[0];
		const double& a = iComponent->get_parameters()[0];
		const double& b = iComponent->get_parameters()[1];
		// get lowest and highest x and y
		double xMin = pos.x - a;
		double xMax = pos.x + a;
		double yMin = pos.y - b;
		double yMax = pos.y + b;
		unsigned long long indexXMin = floor(xMin / cellLength);
		unsigned long long indexXMax = floor(xMax / cellLength);
		unsigned long long indexYMin = floor(yMin / cellLength);
		unsigned long long indexYMax = floor(yMax / cellLength);
		// get grid cells
		if (indexXMin != indexXMax) {
			while (indexXMin <= indexXMax) {
				double xL = boost::algorithm::clamp(cellLength*indexXMin, xMin, xMax);
				double xR = boost::algorithm::clamp(cellLength*(indexXMin + 1), xMin, xMax);
				double yL = sqrt(b*b * (1 - pow((xL - pos.x) / a, 2)));
				double yR = sqrt(b*b * (1 - pow((xR - pos.x) / a, 2)));
				indexYMin = std::min(floor((pos.y - yL) / cellLength), floor((pos.y - yR) / cellLength));
				indexYMax = std::max(floor((pos.y + yL) / cellLength), floor((pos.y + yR) / cellLength));
				while (indexYMin <= indexYMax) {
					unsigned long long index = std::min(resolution - 1, indexXMin) * resolution + std::min(resolution - 1, indexYMin);
					assignedCells.insert(cells[index]);
					cells[index]->add_component(iComponent);
					++indexYMin;
				}
				++indexXMin;
			}
			// in case the ellipse does only occupy one x-cell but multiple y-cells
		}
		else if (indexYMin != indexYMax) {
			while (indexYMin <= indexYMax) {
				unsigned long long index = std::min(resolution - 1, indexXMin) * resolution + std::min(resolution - 1, indexYMin);
				assignedCells.insert(cells[index]);
				cells[index]->add_component(iComponent);
				++indexYMin;
			}
			// in case the ellipse is inside a single grid
		}
		else {
			unsigned long long index = std::min(resolution - 1, indexXMin) * resolution + std::min(resolution - 1, indexYMin);
			assignedCells.insert(cells[index]);
			cells[index]->add_component(iComponent);
		}
	}
			break;
	case 3: {
		// get position and parameters
		ofVec2d& posA = iComponent->get_positions()[0];
		ofVec2d& posB = iComponent->get_positions()[1];
		// get lowest and largest x and y
		double xMin = std::min(posA.x, posB.x);
		double xMax = std::max(posA.x, posB.x);
		double yMin = std::min(posA.y, posB.y);
		double yMax = std::max(posA.y, posB.y);
		// get grid cells
		unsigned long long xA = floor(xMin / cellLength);
		unsigned long long xB = floor(xMax / cellLength);
		unsigned long long yA = floor(yMin / cellLength);
		unsigned long long yB = floor(yMax / cellLength);
		while (xMin < xMax) {
			unsigned long long x = floor(xMin / cellLength);
			unsigned long long indexA = x * resolution + yA;
			unsigned long long indexB = x * resolution + yB;
			assignedCells.insert(cells[indexA]);
			assignedCells.insert(cells[indexB]);
			cells[indexA]->add_component(iComponent);
			cells[indexB]->add_component(iComponent);
			xMin += cellLength;
		}
		while (yMin < yMax) {
			unsigned long long y = floor(yMin / cellLength);
			unsigned long long indexA = xA * resolution + y;
			unsigned long long indexB = xB * resolution + y;
			assignedCells.insert(cells[indexA]);
			assignedCells.insert(cells[indexB]);
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