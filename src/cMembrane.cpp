/*
 * cMembrane.cpp
 *
 *  Created on: May 29, 2017
 *      Author: siegbahn
 */

#include "cMembrane.h"


/***************************
 * membrane part
 ***************************/

membrane_part::membrane_part(
        sGlobalVars &iGlobals,
        cell_base &iCell,
        double iX1, double iY1,
        double iX2, double iY2,
        std::set<stokes::functor *> &iFunctors
) : membrane_part_base(iGlobals, iCell, iX1, iY1, iX2, iY2) {
    update_normal();
};

membrane_part::~membrane_part() {
    //globals.grid->unregister_component(this);
    delete associatedVisualObj;
    associatedVisualObj = nullptr;
};

Eigen::Vector3d &membrane_part::get_normal() {
    return normal;
}

void membrane_part::obtain_visualObjs(std::vector<visual_base *> &iVisualObjs) {
    iVisualObjs.push_back(associatedVisualObj);
}

void membrane_part::make_timeStep(const double &dT) {
    if (solver) {
        //rigidBody->do_timeStep(dT);
    } else {
        std::cout << "membrane_part w/o RigidBody\n";
    }
}

double membrane_part::get_length() {
    return (get_positions()[0] - get_positions()[1]).norm();
}

void membrane_part::update_normal() {
    normal = (Eigen::Vector3d(0, 0, -1).cross(positions[1] - positions[0])).normalized();
};


/***************************
 * membrane container
 ***************************/

// circular membrane
membrane_container::membrane_container(
        sGlobalVars &iGlobals,
        cell_base &iCell
) : cellcomponents_base(iGlobals, iCell) {
    update_area();
    update_length();
};

membrane_container::~membrane_container() {
    for (auto &it : parts) {
        delete it;
        it = nullptr;
    }
}

/* calculate area */
void membrane_container::update_area() {
    //if(!area.isUpdated()) {
    double temp = 0;
    /* calculate 2D volume aka the area */
    for (auto &it : parts) {
        auto &posA = it->get_positions()[0];
        auto &posB = it->get_positions()[1];
        temp += -1 * posB(0) * posA(1) + posA(0) * posB(1);
    };
    area = temp;
}

/* calculate length */
void membrane_container::update_length() {
    //if (!length.isUpdated()) {
    length = 0;
    for (auto &_it : parts) {
        auto _l = _it->get_length();
        length += _l;
    }
}

/* */
void membrane_container::obtain_visualObjs(std::vector<visual_base *> &oVisualComponents) {
    for (auto &it : parts) {
        it->obtain_visualObjs(oVisualComponents);
    }
}

/* get volume */
double &membrane_container::get_area() {
    update_area();
    return area;
}

double &membrane_container::get_length() {
    update_length();
    return length;
}

std::vector<membrane_part_base *> &membrane_container::get_parts() {
    return parts;
}

/*
 * do time step, handles mostly the removal of obselete membrane parts
 */
void membrane_container::make_timeStep(const double &dT) {
    // run through all membrane parts
    auto _it = parts.begin();
    while (_it != parts.end()) {
        // check for linkers
        auto &_linkers = (*_it)->get_connectedLinkers();
        if (_linkers.size() < 2) {
            membrane_part_base *_neigh = (*_it)->get_neighbours().first;
            if (_neigh->get_connectedLinkers().size() < 2) {

                /*
                // get position
                auto _posAP = (*_it)->get_sharedPositions()[0];
                auto _posBP = (*_it)->get_sharedPositions()[1];
                auto _posAN = _neigh->get_sharedPositions()[0];
                auto _posBN = _neigh->get_sharedPositions()[1];
                Eigen::Vector3d *_sharedPos, _posA, _posB;
                membrane_part_base *_neighA, _neighB;
                if (_posAP == _posAN) {
                    _sharedPos = _posAP;
                    _posA = _posBP;
                    _posB = _posBN;
                } else if (_posAP == _posBN) {
                    _sharedPos = _posAP;
                    _posA = _posBP;
                    _posB = _posAN;
                } else if (_posBP == _posAN) {
                    _sharedPos = _posBP;
                    _posA = _posAP;
                    _posB = _posBN;
                } else if (_posBP == _posBN) {
                    _sharedPos = _posBP;
                    _posA = _posAP;
                    _posB = _posAN;
                } else {
                    _sharedPos = nullptr;
                }

                if (_sharedPos) {
                    delete _sharedPos;
                    _sharedPos = nullptr;
                    // set neighbour data
                }*/
                _neigh->set_neighbours({_neigh->get_neighbours().first, (*_it)->get_neighbours().second});
                _neigh->set_sharedPositions({_neigh->get_sharedPositions().first, (*_it)->get_sharedPositions().second});

                // erase this element
                _it = parts.erase(_it);
            }
        } else if (_linkers.size() > 2) {
            std::cout << "Warning : Membrane part has more than two linkers!\n";
        }
        _it++;
    }
}

/***************************
 * arc membrane
 ***************************/

arc_membrane_part::arc_membrane_part(
        sGlobalVars &iGlobals,
        cell_base &iCell,
        double iX1,
        double iY1,
        double R,
        double angleB,
        double angleE,
        std::set<stokes::functor *> &iFunctors
) :
        membrane_part_base(iGlobals, iCell) {
    positions.clear();
    parameters.clear();
    positions.emplace_back(Eigen::Vector3d(iX1, iY1, 0));
    parameters.emplace_back(R);
    parameters.emplace_back(angleB);
    parameters.emplace_back(angleE);
    associatedVisualObj = new visual_arcCircle(this);
    associatedVisualObj->set_color(0.0, 0.0, 0.0);
    associatedVisualObj->set_fillColor(0.0, 0.0, 0.0);
    //globals.grid->register_component(this);
}

arc_membrane_part::~arc_membrane_part() {

}

Eigen::Vector3d arc_membrane_part::get_normal(const double &deg) {
    return Eigen::Vector3d(cos(deg), sin(deg), 0);
}

void arc_membrane_part::make_timeStep(const double &dT) {

}

double arc_membrane_part::get_length() {
    auto _diff = std::abs(get_parameters()[2] - get_parameters()[1]);
    return get_parameters()[0] * (_diff);
}

void arc_membrane_part::obtain_visualObjs(std::vector<visual_base *> &iVisualObjs) {
    iVisualObjs.push_back(associatedVisualObj);
}
