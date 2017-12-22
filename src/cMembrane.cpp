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
) :
        membrane_part_base(iGlobals, iCell) {
    positions.clear();
    positions.emplace_back(Eigen::Vector3d(iX1, iY1, 0));
    positions.emplace_back(Eigen::Vector3d(iX2, iY2, 0));
    associatedVisualObj = new visual_line(this);
    associatedVisualObj->set_color(0.0, 0.0, 0.0);
    associatedVisualObj->set_fillColor(0.0, 0.0, 0.0);
    update_normal();
};

membrane_part::membrane_part(
        sGlobalVars &iGlobals,
        cell_base &iCell,
        const Eigen::Vector3d &iPosS,
        const Eigen::Vector3d &iPosE,
        std::set<stokes::functor *> &iFunctors
) :
        membrane_part_base(iGlobals, iCell) {
    positions.clear();
    positions.emplace_back(iPosS);
    positions.emplace_back(iPosE);
    associatedVisualObj = new visual_line(this);
    associatedVisualObj->set_color(0.0, 0.0, 0.0);
    associatedVisualObj->set_fillColor(0.0, 0.0, 0.0);
    update_normal();
};

membrane_part::~membrane_part() {
    //globals.grid->unregister_component(this);
    delete associatedVisualObj;
    associatedVisualObj = nullptr;
};

/*
void membrane_part::set_sharedPositions(const std::pair<Vector3d *, Vector3d *> &iSharedPos) {
    sharedPositions = iSharedPos;
}

std::pair<Eigen::Vector3d *, Eigen::Vector3d *> &membrane_part::get_sharedPositions() {
    return sharedPositions;
}
*/

Eigen::Vector3d &membrane_part::get_normal() {
    return normal;
}

void membrane_part::obtain_visualObjs(std::vector<visual_base *> &iVisualObjs) {
    iVisualObjs.push_back(associatedVisualObj);
    nextLinker()->obtain_visualObjs(iVisualObjs);
}

void membrane_part::make_timeStep(const double &dT) {
    positions[0] = *vPrevLinker->referencePos();
    positions[1] = *vNextLinker->referencePos();
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

Eigen::Vector3d arc_membrane_part::get_normal(const double &iDeg) {
    return Eigen::Vector3d(cos(iDeg), sin(iDeg), 0);
}

void arc_membrane_part::make_timeStep(const double &dT) {

}

double arc_membrane_part::get_length() {
    auto _diff = std::abs(get_parameters()[2] - get_parameters()[1]);
    return get_parameters()[0] * _diff;
}

void arc_membrane_part::obtain_visualObjs(std::vector<visual_base *> &iVisualObjs) {
    iVisualObjs.push_back(associatedVisualObj);
}

/***************************
 * hyperbolic membrane
 ***************************/

hyperbolic_membrane_part::hyperbolic_membrane_part(
        sGlobalVars &iGlobals,
        cell_base &iCell,
        const Eigen::Vector3d &iPos,
        double iD,
        double iA, double iB,
        double iT1, double iT2,
        double iDir,
        std::set<stokes::functor *> &iFunctors
) : membrane_part_base(iGlobals, iCell) {
    positions.clear();
    positions.push_back(iPos);
    parameters = {iD, iA, iB, iT1, iT2, iDir};
    associatedVisualObj = new visual_hyperbola(this);
    associatedVisualObj->set_color(0.0, 0.0, 0.0);
    associatedVisualObj->set_fillColor(0.0, 0.0, 0.0);
}

hyperbolic_membrane_part::~hyperbolic_membrane_part() {

}

double hyperbolic_membrane_part::get_length() {
    if (!updatedLength) {
        update_curveSegments();
        curveSegmentLenghts.clear();
        auto _s = 0.0d;
        for (unsigned long _i = 1; _i < curveSegments.size(); _i++) {
            auto _l = (curveSegments[_i - 1] - curveSegments[_i]).norm();
            _s += _l;
            curveSegmentLenghts.push_back(_l);
        }
        updatedLength = true;
    }
    return length;
}

void hyperbolic_membrane_part::obtain_visualObjs(std::vector<visual_base *> &iVisualObjs) {
    iVisualObjs.push_back(associatedVisualObj);
}

void hyperbolic_membrane_part::make_timeStep(const double &dT) {
    updatedCurveSegements = false;
    updatedLength = false;
}

void hyperbolic_membrane_part::update_curveSegments() {
    if (!updatedCurveSegements) {
        curveSegments.clear();
        auto &_a = parameters[1];
        auto &_b = parameters[2];
        auto &_t1 = parameters[3];
        auto &_t2 = parameters[4];
        unsigned _resolution = 200;

        auto _min = min(_t1, _t2);
        auto _max = max(_t1, _t2);
        auto _step = abs(_t1 - _t2) / _resolution;

        while (_min <= _max) {
            auto _x = Eigen::Vector3d(_a * cosh(_min), _b * cosh(_min), 0);
            curveSegments.push_back(_x);
            _min = min(_min + _step, _max);
        }
        updatedCurveSegements = true;
    }
}

Eigen::Vector3d hyperbolic_membrane_part::get_normal(double iS) {
    return get_normal(get_segmentId(iS).first);
}

Eigen::Vector3d hyperbolic_membrane_part::get_normal(unsigned iId) {
    auto &_t1 = parameters[3];
    auto &_t2 = parameters[4];
    Eigen::Vector3d _seg = curveSegments[iId + 1] - curveSegments[iId];
    if (_t1 < _t2) {
        return _seg.cross(Eigen::Vector3d(0, 0, -1)).normalized();
    } else {
        return _seg.cross(Eigen::Vector3d(0, 0, 1)).normalized();
    }
}

std::pair<unsigned, double> hyperbolic_membrane_part::get_segmentId(double iS) {
    auto _s = 0.0d;
    unsigned _i = 0;
    while (_s + curveSegmentLenghts[_i] <= iS && _i < curveSegmentLenghts.size()) {
        _s += iS;
        _i++;
    }
    return {_i, _s - iS};
}

const Eigen::Vector3d &hyperbolic_membrane_part::get_segment(double iS) {
    return curveSegments[get_segmentId(iS).first];
}

const Eigen::Vector3d &hyperbolic_membrane_part::get_segment(unsigned iId) {
    return curveSegments[iId];
}

