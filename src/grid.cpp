#include "grid.h"

using namespace grid;

/*
* The grid borders only exist as visual component holders so one can see the grid borders.
*/

border::border(
        double iX1,
        double iY1,
        double iX2,
        double iY2
) : base() {
    positions.clear();
    positions.emplace_back(Eigen::Vector3d(iX1, iY1, 0));
    positions.emplace_back(Eigen::Vector3d(iX2, iY2, 0));
    associatedVisualObj = new visual_line(this);
    associatedVisualObj->set_color(0, 0, 0, 1);
    associatedVisualObj->set_fillColor(0, 0, 0, 1);
}

border::~border() {
    delete associatedVisualObj;
    associatedVisualObj = nullptr;
}

void border::obtain_visualObjs(std::vector<visual_base *> &iVisualObjs) {
    iVisualObjs.push_back(associatedVisualObj);
}

/*
* indicator to show intersections from the line sweep
*/

intersectIndicator::intersectIndicator(const Eigen::Vector3d &iPos) {
    positions.clear();
    positions.emplace_back(iPos);
    associatedVisualObj = new visual_ellipse(this);
    associatedVisualObj->set_color(1, 0, 1);
    associatedVisualObj->set_fillColor(1, 0, 1);
    parameters.push_back(2);
    parameters.push_back(2);
}

intersectIndicator::~intersectIndicator() {
    delete associatedVisualObj;
    associatedVisualObj = nullptr;
}

void intersectIndicator::obtain_visualObjs(std::vector<visual_base *> &iVisualObjs) {
    iVisualObjs.push_back(associatedVisualObj);
}


/*
* The grid cells store which cell components are inside their boundaries. The stored cell components are updated after
 * each time step. All grid cells are stored in the class 'grid' and form the complete grid
*/

cell::cell(
        bool &iShowGrid,
        bool &iShowGridOccupation,
        double iX1,
        double iY1,
        double iX2,
        double iY2
) :
        base(),
        isOccupied(false),
        showGrid(iShowGrid),
        showGridOccupation(iShowGridOccupation) {
    borders.clear();
    borders.push_back(new border(iX1, iY1, iX1, iY2));
    borders.push_back(new border(iX1, iY2, iX2, iY2));
    borders.push_back(new border(iX2, iY2, iX2, iY1));
    borders.push_back(new border(iX2, iY1, iX1, iY1));
    positions.clear();
    positions.emplace_back(Eigen::Vector3d(iX1, iY1, 0));
    positions.emplace_back(Eigen::Vector3d(iX1, iY2, 0));
    positions.emplace_back(Eigen::Vector3d(iX2, iY2, 0));
    positions.emplace_back(Eigen::Vector3d(iX2, iY1, 0));
    associatedVisualObj = new visual_rectangle(this);
    associatedVisualObj->set_color(0, 0, 1, 1);
    associatedVisualObj->set_fillColor(0, 0, 1, 1);
}

cell::~cell() {
    for (auto _it : borders) {
        delete _it;
        _it = nullptr;
    }
}

std::set<base *> &cell::get_components() {
    return components;
}

/*
 * Function which is used to calculate and obtain the intersection information of an arc circle and a segemented line
 *
 * Arguments:
 * 1: cell component (Line) <pointer>
 * 2: cell component (arcCircle) <pointer>
 *
 * Return Value:
 * 1: std::pair<
 *  1: cell component <pointer>
 *  2: normal vector from line to circle center <Eigen::Vector3d>>
 */

Eigen::Vector3d *cell::obtain_intersectingArcCircleLine(base *iRef, base *iCom) {
    // get data
    auto &_posRef = iRef->get_positions();
    auto &_posCom = iRef->get_positions();
    auto &_R = iRef->get_parameters()[0];
    auto &_arcBegin = iRef->get_parameters()[1];
    auto &_arcEnd = iRef->get_parameters()[2];
    // get distances from circle center
    auto _distA = (_posCom[0] - _posRef[0]).norm();
    auto _distB = (_posCom[0] - _posRef[1]).norm();
    double _min;
    double _max;
    unsigned _i;
    if (_distA < _distB) {
        _min = _distA;
        _max = _distB;
        _i = 1;
    } else {
        _min = _distB;
        _max = _distA;
        _i = 0;
    }
    // check if segement crosses circle
    if (_min <= _R && _max > _R) {
        Eigen::Vector3d _diff(_posRef[1] - _posRef[0]);
        // create parameterized line and project sphere center onto it
        Eigen::ParametrizedLine<double, 3> _line(_posRef[0], _diff.normalized());
        // get projection of circle center onto the line
        auto _proj = _line.projection(_posCom[0]);
        // get direction vector
        Eigen::Vector3d _intersectionDir = (_posRef[_i] - _proj).normalized();
        // check if point lies on arc
        auto _angle = bmath::angleVector2d(_intersectionDir(0), _intersectionDir(1));
        if (_angle >= _arcBegin && _angle <= _arcEnd) {
            // calculate intersection point
            auto _a = (_proj - _posCom[0]).norm();
            auto _u = sqrt(_R * _R - _a * _a);
            return new Eigen::Vector3d(_proj + _u * _intersectionDir);
        }
        return nullptr;
    }
    return nullptr;
}

/*
 * Function which is used to calculate and obtain the intersection information of a circle and a segemented line
 *
 * Arguments:
 * 1: cell component (Line) <pointer>
 * 2: cell component (Circle) <pointer>
 *
 * Return Value:
 * 1: std::pair<
 *  1: cell component <pointer>
 *  2: normal vector from line to circle center <Eigen::Vector3d>>
 */

Eigen::Vector3d *cell::obtain_intersectingCircleLine(base *iRef, base *iCom) {
    // get data
    auto &posRef = iRef->get_positions();
    auto &posCom = iCom->get_positions();
    auto &parCom = iCom->get_parameters();
    // get line direction
    Eigen::Vector3d diff(posRef[1] - posRef[0]);
    // create parameterized line and project sphere center onto it
    Eigen::ParametrizedLine<double, 3> line(posRef[0], diff.normalized());
    // get projection of circle center onto the line
    auto proj = line.projection(posCom[0]);
    Eigen::Vector3d projDiff(proj - posCom[0]);
    // check if projection lies inside the circle
    if (projDiff.norm() < parCom[0]) {
        // check if any end lies inside the circle
        auto diffA = posRef[0] - posCom[0];
        auto diffB = posRef[1] - posCom[0];
        if (diffA.norm() <= parCom[0] && diffB.norm() <= parCom[0]) {
            return new Eigen::Vector3d(projDiff.normalized());
        } else if (diffA.norm() <= parCom[0]) {
            return new Eigen::Vector3d(diffA.normalized());
        } else if (diffB.norm() <= parCom[0]) {
            return new Eigen::Vector3d(diffB.normalized());
        } else {
            // check if the end points and projection difference vectors are antiparralel
            if ((posRef[0] - proj).dot(posRef[1] - proj) < 0) {
                return new Eigen::Vector3d(projDiff.normalized());
            }
        }
    }
    return nullptr;
}

/*
 * Function which is used to calculate and obtain the intersection information of two segmented lines
 *
 * Arguments:
 * 1: cell component (Line) <pointer>
 * 2: cell component (Line) <pointer>
 *
 * Return Value:
 * 1: std::pair<
 *  1: cell component <pointer>
 *  2: std::pair<
 *   1: normal vector from line to circle center <Eigen::Vector3d>>
 *   2:
 */

Eigen::Vector3d *cell::obtain_intersectingLineLine(base *iRef, base *iCom) {
    auto &posRef = iRef->get_positions();
    auto &posCom = iCom->get_positions();

    auto unitDiffRef = (posRef[1] - posRef[0]).normalized();
    auto unitDiffCom = (posCom[1] - posCom[0]).normalized();

    auto rcs = unitDiffRef[0] * unitDiffCom[1] - unitDiffRef[1] * unitDiffCom[0];

    if (abs(rcs) >= std::numeric_limits<double>::min()) {
        auto diffPos = posCom[0] - posRef[0];
        auto t = (diffPos[0] * unitDiffCom[1] - diffPos[1] * unitDiffCom[0]) / rcs;
        auto u = (diffPos[0] * unitDiffRef[1] - diffPos[1] * unitDiffRef[0]) / rcs;
        bool test = false;
        if (0 <= t <= 1 && 0 <= u <= 1) {
            auto ret = new Eigen::Vector3d(posRef[0] + t * unitDiffRef);
            auto a = bmath::isInBoundsC((*ret)[0], posRef[0][0], posRef[1][0]);
            auto b = bmath::isInBoundsC((*ret)[1], posRef[0][1], posRef[1][1]);
            auto c = bmath::isInBoundsC((*ret)[0], posCom[0][0], posCom[1][0]);
            auto d = bmath::isInBoundsC((*ret)[1], posCom[0][1], posCom[1][1]);
            if (a && b && c && d) { return ret; };
        }
        return nullptr;
    }
    return nullptr;;
}

/*
 * Function which pushes the pointer to the visual elements of the grid cell into the handed vector
 */

void cell::obtain_visualObjs(std::vector<visual_base *> &iVisualObjs) {
    if (showGridOccupation && !components.empty()) {
        iVisualObjs.push_back(associatedVisualObj);
    }
    if (showGrid) {
        for (auto &it : borders) {
            it->obtain_visualObjs(iVisualObjs);
        }
    }
}

/*
 * Function which handles the update of the intersection information by differentiating between intersection cases
 * e.g. line-circle, line-line, circle-circle
 */

bool cell::obtain_intersecting(base *iComponentA, base *iComponentB) {
    bool ret = false;

    // get data of reference and neighbor object object
    auto &typeA = iComponentA->get_visualObj()->get_type();
    auto &typeB = iComponentB->get_visualObj()->get_type();
    auto &posA = iComponentA->get_positions();
    auto &parA = iComponentA->get_parameters();
    auto &posB = iComponentB->get_positions();
    auto &parB = iComponentB->get_parameters();

    if (iComponentA != iComponentB) {
        // handle two circles
        if (typeA == typeB && typeA == 2) {
            // check whether the two circles overlap
            if ((posA[0] - posB[0]).norm() < parA[0] + parB[0]) {
                Eigen::Vector3d v = posB[0] - posA[0];
                v.normalize();
                iComponentA->add_intersector(iComponentB, v);
                iComponentB->add_intersector(iComponentA, -v);
                ret = true;
            }
        }
            // handle circle and line like
        else if (typeA == 1 && typeB == 2) {
            auto temp = obtain_intersectingCircleLine(iComponentA, iComponentB);
            if (temp) {
                iComponentA->add_intersector(iComponentB, *temp);
                iComponentB->add_intersector(iComponentA, -*temp);
                ret = true;
            }
            delete temp;
            temp = nullptr;
        } else if (typeB == 1 && typeA == 2) {
            auto temp = obtain_intersectingCircleLine(iComponentB, iComponentA);
            if (temp) {
                iComponentA->add_intersector(iComponentB, *temp);
                iComponentB->add_intersector(iComponentA, *temp);
                ret = true;
            }
            delete temp;
            temp = nullptr;

        } // handle arc and line
        else if (typeB == 1 && typeA == 5) {
            auto temp = obtain_intersectingCircleLine(iComponentB, iComponentA);
            if (temp) {
                iComponentA->add_intersector(iComponentB, *temp);
                iComponentB->add_intersector(iComponentA, *temp);
                ret = true;
            }
            delete temp;
            temp = nullptr;
        } else if (typeA == 1 && typeB == 5) {
            auto temp = obtain_intersectingCircleLine(iComponentA, iComponentB);
            if (temp) {
                iComponentA->add_intersector(iComponentB, *temp);
                iComponentB->add_intersector(iComponentA, *temp);
                ret = true;
            }
            delete temp;
            temp = nullptr;
        }

            // handle line objects
        else if (
                (typeA == 1 || typeA == 3 || typeA == 4)
                && (typeB == 1 || typeB == 3 || typeB == 4)
                ) {
            auto temp = obtain_intersectingLineLine(iComponentA, iComponentB);
            if (temp) {
                iComponentA->add_intersector(iComponentB, *temp);
                iComponentB->add_intersector(iComponentA, *temp);
                ret = true;
            }
            delete temp;
            temp = nullptr;
        }
    }

    return
            ret;
}

/*
 * update all intersections inside the grid cell
 */

void cell::update_intersecting() {
    /*for (auto &it : components) {
        it->clear_intersectors();
    }*/
    bool intersection = false;
    for (auto &itA : components) {
        for (auto &itB : components) {
            if (itA != itB) {
                bool ignore = ignore::n::isIgnored(itA->get_typeHash(), itB->get_typeHash());
                if (
                        !ignore &&
                        !itA->isIntersectorChecked(itB) &&
                        !itB->isIntersectorChecked(itA)) {
                    intersection = intersection || obtain_intersecting(itA, itB);
                }
            }
        }
    }
    if (showGridOccupation) {
        if (intersection) {
            associatedVisualObj->set_color(1, 0, 1, 1);
            associatedVisualObj->set_fillColor(1, 0, 1, 1);
        } else {
            associatedVisualObj->set_color(0, 0, 1, 1);
            associatedVisualObj->set_fillColor(0, 0, 1, 1);
        }
    }
}

/*
 * removes a component of type base from the cell
 */

void cell::remove_component(base *iComponent) {
    if (iComponent) {
        components.erase(iComponent);
    }
}

/*
 * adds a component to the cell list
 */

void cell::add_component(base *iComponent) {
    components.insert(iComponent);
}

/*
 * placeholder for cell reset but can probably be deleted
 */

void cell::reset() {

}

/***************************
* grid_base
***************************/

container::container(mygui::gui *&iGuiBase, double iSideLength) :
        guiBase(iGuiBase),
        sideLength(iSideLength),
        guiGroup(guiBase->register_group("Grid")),
        showGrid(guiGroup->register_setting<bool>("Show grid", true, false)),
        showGridOccupation(guiGroup->register_setting<bool>("show Occ", true, false)),
        resolution(guiGroup->register_setting<unsigned>("Resolution", false, 2, 250, 100)),
        doLineSweep(true) {
    create_cells();
}

container::~container() {
    for (auto _it : cells) {
        delete _it;
        _it = nullptr;
    }
    for (auto _it : intersectIndicators) {
        delete _it;
        _it = nullptr;
    }
}

void container::obtain_visualObjs(std::vector<visual_base *> &iVisualObjs) {
    if (showGrid || showGridOccupation) {
        for (auto &_it : cells) {
            _it->obtain_visualObjs(iVisualObjs);
        }
        for (auto &_it : intersectIndicators) {
            _it->obtain_visualObjs(iVisualObjs);
        }
    }
}

void container::register_component(base *iComponent) {
    components.insert(iComponent);
    typeCheckQueue.push_back(iComponent);
}

void container::unregister_component(base *iComponent) {
    for (auto &it : iComponent->get_gridCells()) {
        it->remove_component(iComponent);
    }
    components.erase(iComponent);
}

/*
 *  updates and finds the cell components which are located inside the grid cell
 */

void container::update_component(base *iComponent) {
    double _cellLength = sideLength / (double) resolution;
    // remove entries from old cells first
    for (auto &_it : iComponent->get_gridCells()) {
        _it->remove_component(iComponent);
    }
    //assign new cells
    std::set<cell *> _assignedCells;
    switch (iComponent->get_visualObj()->get_type()) {
        // line
        case 1: {
            auto &_posA = iComponent->get_positions()[0];
            auto &_posB = iComponent->get_positions()[1];

            // get slope
            auto _m = std::numeric_limits<double>::infinity();
            if (_posA(0) != _posB(0)) {
                _m = (_posA(1) - _posB(1)) / (_posA(0) - _posB(0));
            }
            // get f(0)
            double _f0 = (_posA(1) - _m * _posA(0));
            double _f0Red = _f0 / _cellLength;
            // get mins and maxs
            auto _yMin = std::min(_posA(1), _posB(1));
            auto _yMax = std::max(_posA(1), _posB(1));
            auto _xMin = std::min(_posA(0), _posB(0));
            auto _xMax = std::max(_posA(0), _posB(0));
            auto _indexXMin = (unsigned) floor(_xMin / _cellLength);
            auto _indexXMax = (unsigned) floor(_xMax / _cellLength);
            auto _indexYMin = (unsigned) floor(_yMin / _cellLength);
            auto _indexYMax = (unsigned) floor(_yMax / _cellLength);
            // get cells
            if (abs(_m) < 1) {
                while (_indexXMin <= _indexXMax) {
                    // calculate y value
                    auto _indexTestT = (unsigned) floor(_f0Red + _m * _indexXMin);
                    auto _indexTestB = (unsigned) boost::algorithm::clamp(floor(_f0Red + _m * (_indexXMin + 1)),
                                                                          _indexYMin, _indexYMax);
                    unsigned _indexT =
                            std::min(resolution - 1, _indexXMin) * resolution + std::min(resolution - 1, _indexTestT);
                    unsigned _indexB =
                            std::min(resolution - 1, _indexXMin) * resolution + std::min(resolution - 1, _indexTestB);
                    _assignedCells.insert(cells[_indexT]);
                    cells[_indexT]->add_component(iComponent);
                    if (_indexT != _indexB) {
                        _assignedCells.insert(cells[_indexB]);
                        cells[_indexB]->add_component(iComponent);
                    }
                    ++_indexXMin;
                }
            } else {
                while (_indexYMin <= _indexYMax) {
                    if (abs(_m) < std::numeric_limits<double>::max()) {
                        // calculate x value
                        auto _indexTestL = (unsigned) floor((_indexYMin - _f0Red) / _m);
                        auto _indexTestR = (unsigned) boost::algorithm::clamp(floor(((_indexYMin + 1) - _f0Red) / _m),
                                                                              _indexXMin, _indexXMax);
                        unsigned _indexL =
                                std::min(resolution - 1, _indexTestL) * resolution +
                                std::min(resolution - 1, _indexYMin);
                        unsigned _indexR =
                                std::min(resolution - 1, _indexTestR) * resolution +
                                std::min(resolution - 1, _indexYMin);
                        _assignedCells.insert(cells[_indexL]);
                        cells[_indexL]->add_component(iComponent);
                        if (_indexL != _indexR) {
                            _assignedCells.insert(cells[_indexR]);
                            cells[_indexR]->add_component(iComponent);
                        }
                    } else {
                        // take care of vertical lines
                        unsigned _index =
                                std::min(resolution - 1, _indexXMin) * resolution +
                                std::min(resolution - 1, _indexYMin);
                        _assignedCells.insert(cells[_index]);
                        cells[_index]->add_component(iComponent);
                    }
                    ++_indexYMin;
                }
            }
        }
            break;
            // ellipse
        case 2: {
            // get position and parameters
            auto &_pos = iComponent->get_positions()[0];
            auto &_a = iComponent->get_parameters()[0];
            auto &_b = iComponent->get_parameters()[1];
            // get lowest and highest x and y
            auto _xMin = _pos(0) - _a;
            auto _xMax = _pos(0) + _a;
            auto _yMin = _pos(1) - _b;
            auto _yMax = _pos(1) + _b;
            auto _indexXMin = (unsigned) floor(_xMin / _cellLength);
            auto _indexXMax = (unsigned) floor(_xMax / _cellLength);
            auto _indexYMin = (unsigned) floor(_yMin / _cellLength);
            auto _indexYMax = (unsigned) floor(_yMax / _cellLength);
            // get grid cells
            if (_indexXMin != _indexXMax) {
                while (_indexXMin <= _indexXMax) {
                    double _xL = boost::algorithm::clamp(_cellLength * _indexXMin, _xMin, _xMax);
                    double _xR = boost::algorithm::clamp(_cellLength * (_indexXMin + 1), _xMin, _xMax);
                    double _yL = sqrt(_b * _b * (1 - pow((_xL - _pos(0)) / _a, 2)));
                    double _yR = sqrt(_b * _b * (1 - pow((_xR - _pos(0)) / _a, 2)));
                    _indexYMin = (unsigned) std::min(floor((_pos(1) - _yL) / _cellLength),
                                                     floor((_pos(1) - _yR) / _cellLength));
                    _indexYMax = (unsigned) std::max(floor((_pos(1) + _yL) / _cellLength),
                                                     floor((_pos(1) + _yR) / _cellLength));
                    while (_indexYMin <= _indexYMax) {
                        unsigned _index =
                                std::min(resolution - 1, _indexXMin) * resolution +
                                std::min(resolution - 1, _indexYMin);
                        _assignedCells.insert(cells[_index]);
                        cells[_index]->add_component(iComponent);
                        ++_indexYMin;
                    }
                    ++_indexXMin;
                }
                // in case the ellipse does only occupy one x-cell but multiple y-cells
            } else if (_indexYMin != _indexYMax) {
                while (_indexYMin <= _indexYMax) {
                    unsigned _index =
                            std::min(resolution - 1, _indexXMin) * resolution + std::min(resolution - 1, _indexYMin);
                    _assignedCells.insert(cells[_index]);
                    cells[_index]->add_component(iComponent);
                    ++_indexYMin;
                }
                // in case the ellipse is inside a single grid
            } else {
                unsigned _index =
                        std::min(resolution - 1, _indexXMin) * resolution + std::min(resolution - 1, _indexYMin);
                _assignedCells.insert(cells[_index]);
                cells[_index]->add_component(iComponent);
            }
        }
            break;
        case 3: {
            // get position and parameters
            auto &_posA = iComponent->get_positions()[0];
            auto &_posB = iComponent->get_positions()[1];
            // get lowest and largest x and y
            double _xMin = std::min(_posA(0), _posB(0));
            double _xMax = std::max(_posA(0), _posB(0));
            double _yMin = std::min(_posA(1), _posB(1));
            double _yMax = std::max(_posA(1), _posB(1));
            // get grid cells
            auto _xA = (unsigned) floor(_xMin / _cellLength);
            auto _xB = (unsigned) floor(_xMax / _cellLength);
            auto _yA = (unsigned) floor(_yMin / _cellLength);
            auto _yB = (unsigned) floor(_yMax / _cellLength);
            while (_xMin < _xMax) {
                auto _x = (unsigned) floor(_xMin / _cellLength);
                unsigned _indexA = _x * resolution + _yA;
                unsigned _indexB = _x * resolution + _yB;
                _assignedCells.insert(cells[_indexA]);
                _assignedCells.insert(cells[_indexB]);
                cells[_indexA]->add_component(iComponent);
                cells[_indexB]->add_component(iComponent);
                _xMin += _cellLength;
            }
            while (_yMin < _yMax) {
                auto _y = (unsigned) floor(_yMin / _cellLength);
                unsigned _indexA = _xA * resolution + _y;
                unsigned _indexB = _xB * resolution + _y;
                _assignedCells.insert(cells[_indexA]);
                _assignedCells.insert(cells[_indexB]);
                _yMin += _cellLength;
            }
        }
            break;
        case 5: {
            // get position and parameters
            auto &_pos = iComponent->get_positions()[0];
            auto &_R = iComponent->get_parameters()[0];
            auto &_angleB = iComponent->get_parameters()[1];
            auto &_angleE = iComponent->get_parameters()[2];
            // get lowest and highest x and y
            auto _xAngleB = _R * cos(_angleB);
            auto _yAngleB = _R * sin(_angleB);
            auto _xAngleE = _R * cos(_angleE);
            auto _yAngleE = _R * sin(_angleE);

            double _xMin, _xMax, _yMin, _yMax;
            if (_xAngleB > _xAngleE) {
                _xMax = _xAngleB;
                _xMin = _xAngleE;
            } else {
                _xMax = _xAngleE;
                _xMin = _xAngleB;
            }
            if (_yAngleB > _yAngleE) {
                _yMax = _yAngleB;
                _yMin = _yAngleE;
            } else {
                _yMax = _yAngleE;
                _yMin = _yAngleB;
            }

            _xMin += _pos(0);
            _xMax += _pos(0);
            _yMin += _pos(1);
            _yMax += _pos(1);
            // get smallest and largest grid cell index
            auto _indexXMin = (unsigned) floor(_xMin / _cellLength);
            auto _indexXMax = (unsigned) floor(_xMax / _cellLength);
            auto _indexYMin = (unsigned) floor(_yMin / _cellLength);
            auto _indexYMax = (unsigned) floor(_yMax / _cellLength);
            // get grid cells
            if (_indexXMin != _indexXMax) {
                while (_indexXMin <= _indexXMax) {
                    double _xL = boost::algorithm::clamp(_cellLength * _indexXMin, _xMin, _xMax);
                    double _xR = boost::algorithm::clamp(_cellLength * (_indexXMin + 1), _xMin, _xMax);
                    double _yL = sqrt(_R * _R * (1 - pow((_xL - _pos(0)) / _R, 2)));
                    double _yR = sqrt(_R * _R * (1 - pow((_xR - _pos(0)) / _R, 2)));
                    _indexYMin = (unsigned) std::min(floor((_pos(1) - _yL) / _cellLength),
                                                     floor((_pos(1) - _yR) / _cellLength));
                    _indexYMax = (unsigned) std::max(floor((_pos(1) + _yL) / _cellLength),
                                                     floor((_pos(1) + _yR) / _cellLength));
                    while (_indexYMin <= _indexYMax) {
                        unsigned _index =
                                std::min(resolution - 1, _indexXMin) * resolution +
                                std::min(resolution - 1, _indexYMin);
                        _assignedCells.insert(cells[_index]);
                        cells[_index]->add_component(iComponent);
                        ++_indexYMin;
                    }
                    ++_indexXMin;
                }
                // in case the ellipse does only occupy one x-cell but multiple y-cells
            } else if (_indexYMin != _indexYMax) {
                while (_indexYMin <= _indexYMax) {
                    unsigned _index =
                            std::min(resolution - 1, _indexXMin) * resolution + std::min(resolution - 1, _indexYMin);
                    _assignedCells.insert(cells[_index]);
                    cells[_index]->add_component(iComponent);
                    ++_indexYMin;
                }
                // in case the ellipse is inside a single grid
            } else {
                unsigned _index =
                        std::min(resolution - 1, _indexXMin) * resolution + std::min(resolution - 1, _indexYMin);
                _assignedCells.insert(cells[_index]);
                cells[_index]->add_component(iComponent);
            }
        }
            break;
        default : {
            std::cout << "GRID (Warning): unidentified object shape\n";
        }
    }
    iComponent->set_gridCells(_assignedCells);
}

void container::update_components() {
    for (auto &it : components) {
        it->clear_intersectors();
        if (!doLineSweep) {
            update_component(it);
        }
    }
    if (doLineSweep) {
        lineSweep();
    } else {
        for (auto &it : cells) {
            it->update_intersecting();
        }
    }
}

void container::reset() {
    for (auto _it : cells) {
        delete _it;
        _it = nullptr;
    }
    cells.clear();
    intersectIndicators.clear();
    guiGroup->forceVariableUpdate();
    create_cells();
}

void container::create_cells() {
    double stepLength = sideLength / (double) resolution;
    for (unsigned long long i = 0; i < resolution; i++) {
        for (unsigned long long j = 0; j < resolution; j++) {
            double iX1 = stepLength * i;
            double iY1 = stepLength * j;
            double iX2 = stepLength * (i + 1);
            double iY2 = stepLength * (j + 1);
            cells.push_back(new cell(showGrid, showGridOccupation, iX1, iY1, iX2, iY2));
        }
    }
}

void container::lineSweep() {
    // do the line sweep
    auto _intersections = lazySweep::sweep(components);
    // delete all visual intersection indicators
    for (auto _it : intersectIndicators) {
        delete _it;
        _it = nullptr;
    }
    intersectIndicators.clear();
    // create new visuals
    if (showGridOccupation) {
        for (auto &_it : _intersections) {
            intersectIndicators.push_back(new intersectIndicator(_it.pos));
        }
    }
}

void container::typeCheck() {
    if (doLineSweep) {
        for (auto _it : typeCheckQueue) {
            if (_it->get_visualObj()->get_type() != 1) {
                doLineSweep = false;
                std::cout << "Grid: Can't do line sweep, using fallback!";
            }
        }
    }
    typeCheckQueue.clear();
}
