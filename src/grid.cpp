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
    for (auto &it : borders) {
        delete it;
        it = nullptr;
    }
}

std::set<base *> &cell::get_components() {
    return components;
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
    // get vector direction
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
            if (a && b && c && d) {return ret;};
        }
        return nullptr;
    }
    return nullptr;
;
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
                auto v = posB[0] - posA[0];
                v.normalize();
                iComponentA->add_intersector(iComponentB, v);
                iComponentB->add_intersector(iComponentA, -v);
                ret = true;
            }
        }
            // handle circle and line like
        else if ((typeA == 1 || typeA == 3 || typeA == 4) && typeB == 2) {
            auto temp = obtain_intersectingCircleLine(iComponentA, iComponentB);
            if (temp) {
                iComponentA->add_intersector(iComponentB, *temp);
                iComponentB->add_intersector(iComponentA, -*temp);
                ret = true;
            }
            delete temp;
            temp = nullptr;
        } else if ((typeB == 1 || typeB == 3 || typeB == 4) && typeA == 2) {
            auto temp = obtain_intersectingCircleLine(iComponentB, iComponentA);
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
    return ret;
}

/*  */

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
                        !ignore /*&&
                        !itA->isIntersectorChecked(itB) &&
                        !itB->isIntersectorChecked(itA)*/) {
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

/* removes a component of type base from the cell */

void cell::remove_component(base *iComponent) {
    if (iComponent) {
        components.erase(iComponent);
    }
}

/* adds a component to the cell list*/

void cell::add_component(base *iComponent) {
    components.insert(iComponent);
}

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
        resolution(guiGroup->register_setting<unsigned>("Resolution", false, 2, 250, 100)) {
    create_cells();
}

container::~container() {
    for (auto &it : cells) {
        delete it;
        it = nullptr;
    }
}

void container::obtain_visualObjs(std::vector<visual_base *> &iVisualObjs) {
    if (showGrid || showGridOccupation) {
        for (auto &it : cells) {
            it->obtain_visualObjs(iVisualObjs);
        }
    }
}

void container::register_component(base *iComponent) {
    // removed check b/c this function is mostly called in headers and causes problems with virtuality
    //if (iComponent->get_visualObj()) {
    components.insert(iComponent);
    /*} else {
        std::cout << "Could not register object with ID: " << typeid(*iComponent).name()
                  << "\n Visual object is missing";
    }*/
    return void();
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
    double cellLength = sideLength / (double) resolution;
    // remove entries from old cells first
    for (auto &it : iComponent->get_gridCells()) {
        it->remove_component(iComponent);
    }
    //assign new cells
    std::set<cell *> assignedCells;
    switch (iComponent->get_visualObj()->get_type()) {
        // line
        case 1: {
            auto &posA = iComponent->get_positions()[0];
            auto &posB = iComponent->get_positions()[1];

            // get slope
            double m = std::numeric_limits<double>::infinity();
            if (posA(0) != posB(0)) {
                m = (posA(1) - posB(1)) / (posA(0) - posB(0));
            }
            // get f(0)
            double f0 = (posA(1) - m * posA(0));
            double f0Red = f0 / cellLength;
            // get mins and maxs
            auto yMin = std::min(posA(1), posB(1));
            auto yMax = std::max(posA(1), posB(1));
            auto xMin = std::min(posA(0), posB(0));
            auto xMax = std::max(posA(0), posB(0));
            auto indexXMin = (unsigned) floor(xMin / cellLength);
            auto indexXMax = (unsigned) floor(xMax / cellLength);
            auto indexYMin = (unsigned) floor(yMin / cellLength);
            auto indexYMax = (unsigned) floor(yMax / cellLength);
            // get cells
            if (abs(m) < 1) {
                while (indexXMin <= indexXMax) {
                    // calculate y value
                    auto indexTestT = (unsigned) floor(f0Red + m * indexXMin);
                    auto indexTestB = (unsigned) boost::algorithm::clamp(floor(f0Red + m * (indexXMin + 1)),
                                                                         indexYMin, indexYMax);
                    unsigned indexT =
                            std::min(resolution - 1, indexXMin) * resolution + std::min(resolution - 1, indexTestT);
                    unsigned indexB =
                            std::min(resolution - 1, indexXMin) * resolution + std::min(resolution - 1, indexTestB);
                    assignedCells.insert(cells[indexT]);
                    cells[indexT]->add_component(iComponent);
                    if (indexT != indexB) {
                        assignedCells.insert(cells[indexB]);
                        cells[indexB]->add_component(iComponent);
                    }
                    ++indexXMin;
                }
            } else {
                while (indexYMin <= indexYMax) {
                    if (abs(m) < std::numeric_limits<double>::max()) {
                        // calculate x value
                        auto indexTestL = (unsigned) floor((indexYMin - f0Red) / m);
                        auto indexTestR = (unsigned) boost::algorithm::clamp(floor(((indexYMin + 1) - f0Red) / m),
                                                                             indexXMin, indexXMax);
                        unsigned indexL =
                                std::min(resolution - 1, indexTestL) * resolution + std::min(resolution - 1, indexYMin);
                        unsigned indexR =
                                std::min(resolution - 1, indexTestR) * resolution + std::min(resolution - 1, indexYMin);
                        assignedCells.insert(cells[indexL]);
                        cells[indexL]->add_component(iComponent);
                        if (indexL != indexR) {
                            assignedCells.insert(cells[indexR]);
                            cells[indexR]->add_component(iComponent);
                        }
                    } else {
                        // take care of vertical lines
                        unsigned index =
                                std::min(resolution - 1, indexXMin) * resolution + std::min(resolution - 1, indexYMin);
                        assignedCells.insert(cells[index]);
                        cells[index]->add_component(iComponent);
                    }
                    ++indexYMin;
                }
            }
        }
            break;
            // ellipse
        case 2: {
            // get position and parameters
            auto &pos = iComponent->get_positions()[0];
            auto &a = iComponent->get_parameters()[0];
            auto &b = iComponent->get_parameters()[1];
            // get lowest and highest x and y
            auto xMin = pos(0) - a;
            auto xMax = pos(0) + a;
            auto yMin = pos(1) - b;
            auto yMax = pos(1) + b;
            auto indexXMin = (unsigned) floor(xMin / cellLength);
            auto indexXMax = (unsigned) floor(xMax / cellLength);
            auto indexYMin = (unsigned) floor(yMin / cellLength);
            auto indexYMax = (unsigned) floor(yMax / cellLength);
            // get grid cells
            if (indexXMin != indexXMax) {
                while (indexXMin <= indexXMax) {
                    double xL = boost::algorithm::clamp(cellLength * indexXMin, xMin, xMax);
                    double xR = boost::algorithm::clamp(cellLength * (indexXMin + 1), xMin, xMax);
                    double yL = sqrt(b * b * (1 - pow((xL - pos(0)) / a, 2)));
                    double yR = sqrt(b * b * (1 - pow((xR - pos(0)) / a, 2)));
                    indexYMin = (unsigned) std::min(floor((pos(1) - yL) / cellLength),
                                                    floor((pos(1) - yR) / cellLength));
                    indexYMax = (unsigned) std::max(floor((pos(1) + yL) / cellLength),
                                                    floor((pos(1) + yR) / cellLength));
                    while (indexYMin <= indexYMax) {
                        unsigned index =
                                std::min(resolution - 1, indexXMin) * resolution + std::min(resolution - 1, indexYMin);
                        assignedCells.insert(cells[index]);
                        cells[index]->add_component(iComponent);
                        ++indexYMin;
                    }
                    ++indexXMin;
                }
                // in case the ellipse does only occupy one x-cell but multiple y-cells
            } else if (indexYMin != indexYMax) {
                while (indexYMin <= indexYMax) {
                    unsigned index =
                            std::min(resolution - 1, indexXMin) * resolution + std::min(resolution - 1, indexYMin);
                    assignedCells.insert(cells[index]);
                    cells[index]->add_component(iComponent);
                    ++indexYMin;
                }
                // in case the ellipse is inside a single grid
            } else {
                unsigned index =
                        std::min(resolution - 1, indexXMin) * resolution + std::min(resolution - 1, indexYMin);
                assignedCells.insert(cells[index]);
                cells[index]->add_component(iComponent);
            }
        }
            break;
        case 3: {
            // get position and parameters
            auto &posA = iComponent->get_positions()[0];
            auto &posB = iComponent->get_positions()[1];
            // get lowest and largest x and y
            double xMin = std::min(posA(0), posB(0));
            double xMax = std::max(posA(0), posB(0));
            double yMin = std::min(posA(1), posB(1));
            double yMax = std::max(posA(1), posB(1));
            // get grid cells
            auto xA = (unsigned) floor(xMin / cellLength);
            auto xB = (unsigned) floor(xMax / cellLength);
            auto yA = (unsigned) floor(yMin / cellLength);
            auto yB = (unsigned) floor(yMax / cellLength);
            while (xMin < xMax) {
                auto x = (unsigned) floor(xMin / cellLength);
                unsigned indexA = x * resolution + yA;
                unsigned indexB = x * resolution + yB;
                assignedCells.insert(cells[indexA]);
                assignedCells.insert(cells[indexB]);
                cells[indexA]->add_component(iComponent);
                cells[indexB]->add_component(iComponent);
                xMin += cellLength;
            }
            while (yMin < yMax) {
                auto y = (unsigned) floor(yMin / cellLength);
                unsigned indexA = xA * resolution + y;
                unsigned indexB = xB * resolution + y;
                assignedCells.insert(cells[indexA]);
                assignedCells.insert(cells[indexB]);
                yMin += cellLength;
            }
        }
            break;
    }
    iComponent->set_gridCells(assignedCells);
}

void container::update_components() {
    for (auto &it : components) {
        it->clear_intersectors();
        update_component(it);
    }
    for (auto &it : cells) {
        it->update_intersecting();
    }
}

void container::reset() {
    for (auto &it : cells) {
        delete it;
        it = nullptr;
    }
    cells.clear();
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