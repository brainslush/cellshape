#include "grid.h"

using namespace grid;

/***************************
* grid_borders
***************************/

border::border(
        double iX1,
        double iY1,
        double iX2,
        double iY2
) : base() {
    positions.clear();
    positions.push_back(Eigen::Vector3d(iX1, iY1, 0));
    positions.push_back(Eigen::Vector3d(iX2, iY2, 0));
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


/***************************
* grid_cell
***************************/

cell::cell(
        bool &iShowGrid,
        bool &iShowGridOccupation,
        double iX1,
        double iY1,
        double iX2,
        double iY2
) :
        base(),
        showGrid(iShowGrid),
        showGridOccupation(iShowGridOccupation) {
    borders.clear();
    borders.push_back(new border(iX1, iY1, iX1, iY2));
    borders.push_back(new border(iX1, iY2, iX2, iY2));
    borders.push_back(new border(iX2, iY2, iX2, iY1));
    borders.push_back(new border(iX2, iY1, iX1, iY1));
    positions.clear();
    positions.push_back(Eigen::Vector3d(iX1, iY1, 0));
    positions.push_back(Eigen::Vector3d(iX1, iY2, 0));
    positions.push_back(Eigen::Vector3d(iX2, iY2, 0));
    positions.push_back(Eigen::Vector3d(iX2, iY1, 0));
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

std::pair<base *, Eigen::Vector3d> cell::obtain_intersectingCircleLine(base *iRef, base *iCom) {
    // get data
    std::vector<Eigen::Vector3d> &posRef = iRef->get_positions();
    std::vector<Eigen::Vector3d> &posCom = iCom->get_positions();
    std::vector<double> &parCom = iCom->get_parameters();
    // get vector direction
    Eigen::Vector3d diff(posRef[1] - posRef[0]);
    // get length
    double length = diff.norm();
    // create parameterized line and project sphere center onto it
    Eigen::ParametrizedLine<double, 3> line(posRef[0], diff.normalized());
    Eigen::Vector3d proj = line.projection(posCom[0]);
    Eigen::Vector3d projDiff(proj - posCom[0]);

    // check if projection lies on line and inside the sphere
    if (
            (proj - posRef[0]).norm() < length &&
            projDiff.norm() < parCom[0]
            ) {
        return std::make_pair(iCom, projDiff);
    }
    return std::make_pair(iRef, posRef[0]);
}

std::pair<base *, std::pair<Eigen::Vector3d, Eigen::Vector3d>>
cell::obtain_intersectingLineLine(base *iRef, base *iCom) {
    std::vector<Eigen::Vector3d> &posRef = iRef->get_positions();
    std::vector<Eigen::Vector3d> &posCom = iCom->get_positions();

    Eigen::Vector3d &l1S = posRef[0];
    Eigen::Vector3d &l1E = posRef[1];
    Eigen::Vector3d &l2S = posCom[0];
    Eigen::Vector3d &l2E = posCom[1];

    double compareA1, compareA2, compareB1, compareB2, compareD;
    Eigen::Vector3d diffL1 = l1E - l1S;
    Eigen::Vector3d diffL2 = l2E - l2S;

    compareA1 = diffL1(0) * l1S(1) - diffL1(1) * l1S(0);
    compareA2 = diffL1(0) * l2S(1) - diffL1(1) * l2S(0);
    if (compareA1 >= compareA2) {
        compareD = diffL1(0) * diffL2(1) - diffL2(0) * diffL1 (1);
        if (compareA1 - compareA2 <= compareD) {
            compareB1 = diffL2(0) * l2S(1) - diffL2(1) * l2S(0);
            compareB2 = diffL2(0) * l1S(1) - diffL2(1) * l1S(0);
            if (compareB1 <= compareB2) {
                if (compareB2 - compareB1 <= compareD) {
                    diffL1.normalize();
                    diffL2.normalize();
                    return std::make_pair(iCom, std::make_pair(diffL1, diffL2));
                }
            }
        }
    }
    return std::make_pair(iRef, std::make_pair(l1S, l2S));

    /*
    if ((
                (
                        compareC = diffL1(0) * l2S(1) - diffL1(1) * l2S(0);
                        compareA2 <= compareA1
                ) ^
                ((diffL1(0) * l2E(1) - diffL1(1) * l2E(0)) < compareA)
        ) && (
                ((diffL2(0) * l1S(1) - diffL2(1) * l1S(0)) >= compareB) ^
                ((diffL2(0) * l1E(1) - diffL2(1) * l1E(0)) < compareB)
        )) {
        diffL1.normalize();
        diffL2.normalize();
        return std::make_pair(iCom, std::make_pair(diffL1, diffL2));
    }

    float s, t;
    s = (-s1_y * (p0_x - p2_x) + s1_x * (p0_y - p2_y)) / (-s2_x * s1_y + s1_x * s2_y) >= 0;
    s = s1_x * p0_y - s1_y * p0_x >= s1_x * p2_y - s1_y * p2_x;

    s = (-s1_y * (p0_x - p2_x) + s1_x * (p0_y - p2_y)) / (-s2_x * s1_y + s1_x * s2_y) <= 1;
    s = -s1_y * (p0_x - p2_x) + s1_x * (p0_y - p2_y) <= -s2_x * s1_y + s1_x * s2_y;


    t = (s2_x * (p0_y - p2_y) - s2_y * (p0_x - p2_x)) / (-s2_x * s1_y + s1_x * s2_y) >= 0;
    t = s2_x * p0_y - s2_y * p0_x >= s2_x * p2_y - s2_y * p2_x;

    if (s >= 0 && s <= 1 && t >= 0 && t <= 1) {
        // Collision detected
        if (i_x)
            *i_x = p0_x + (t * s1_x);
        if (i_y)
            *i_y = p0_y + (t * s1_y);
        return 1;
    }
     */
}

void cell::obtain_visualObjs(std::vector<visual_base *> &iVisualObjs) {
    if (showGridOccupation && components.size() > 0) {
        iVisualObjs.push_back(associatedVisualObj);
    }
    if (showGrid) {
        for (auto &it : borders) {
            it->obtain_visualObjs(iVisualObjs);
        }
    }
}

bool cell::obtain_intersecting(base *iComponentA, base *iComponentB) {
    bool ret = false;
    std::pair<base *, Eigen::Vector3d> temp;

    // get data of reference and neighbor object object
    unsigned &typeA = iComponentA->get_visualObj()->get_type();
    unsigned &typeB = iComponentB->get_visualObj()->get_type();
    std::vector<Eigen::Vector3d> &posA = iComponentA->get_positions();
    std::vector<double> &parA = iComponentA->get_parameters();
    std::vector<Eigen::Vector3d> &posB = iComponentB->get_positions();
    std::vector<double> &parB = iComponentB->get_parameters();

    if (iComponentA != iComponentB) {
        // handle two circles
        /*
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
        else if ((typeA == 1 || typeA == 3 || typeA == 4) && typeB == 2) {
            temp = obtain_intersectingCircleLine(iComponentA, iComponentB);
            if (temp.first != iComponentA) {
                iComponentA->add_intersector(iComponentB, temp.second);
                iComponentB->add_intersector(iComponentA, -temp.second);
                ret = true;
            }
        } else if ((typeB == 1 || typeB == 3 || typeB == 4) && typeA == 2) {
            temp = obtain_intersectingCircleLine(iComponentB, iComponentA);
            if (temp.first == iComponentA) {
                iComponentA->add_intersector(iComponentB, temp.second);
                iComponentB->add_intersector(iComponentA, temp.second);
                ret = true;
            }
        }

            // handle line objects
        else */if (
                (typeA == 1 || typeA == 3 || typeA == 4)
                && (typeB == 1 || typeB == 3 || typeB == 4)
                ) {
            std::pair<base *, std::pair<Eigen::Vector3d, Eigen::Vector3d>> temp2 = obtain_intersectingLineLine(
                    iComponentA, iComponentB);
            if (temp2.first == iComponentA) {
                iComponentA->add_intersector(iComponentB, temp2.second.first);
                iComponentB->add_intersector(iComponentA, temp2.second.second);
                ret = true;
            }
        }
    }
    return ret;
}

void cell::update_intersecting() {
    std::set<unsigned>::iterator itType;
    for (auto &it : components) {
        it->clear_intersectors();
    }
    bool intersection = false;
    for (auto &itA : components) {
        for (auto &itB : components) {
            if (
                    itA != itB &&
                    itA->get_ignoreIntersect().find(typeid(*itB).hash_code()) == itA->get_ignoreIntersect().end() &&
                    itB->get_ignoreIntersect().find(typeid(*itA).hash_code()) == itB->get_ignoreIntersect().end()
                    ) {
                intersection = intersection || obtain_intersecting(itA, itB);
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

void cell::remove_component(base *iComponent) {
    if (iComponent) {
        components.erase(iComponent);
    }
}

void cell::add_component(base *iComponent) {
    components.insert(iComponent);
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
        resolution(guiGroup->register_setting<unsigned>("Resolution", false, 50, 250, 100)) {
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
    if (iComponent->get_visualObj()) {
        components.insert(iComponent);
    } else {
        std::cout << "Could not register object with ID: " << typeid(*iComponent).name()
                  << "\n Visual object is missing";
    }
}

void container::unregister_component(base *iComponent) {
    for (auto &it : iComponent->get_gridCells()) {
        it->remove_component(iComponent);
    }
    components.erase(iComponent);
}

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
            const Eigen::Vector3d &posA = iComponent->get_positions()[0];
            const Eigen::Vector3d &posB = iComponent->get_positions()[1];

            // get slope
            double m = std::numeric_limits<double>::infinity();
            if (posA(0) != posB(0)) {
                m = (posA(1) - posB(1)) / (posA(0) - posB(0));
            }
            // get f(0)
            double f0 = (posA(1) - m * posA(0));
            double f0Red = f0 / cellLength;
            // get mins and maxs
            double yMin = std::min(posA(1), posB(1));
            double yMax = std::max(posA(1), posB(1));
            double xMin = std::min(posA(0), posB(0));
            double xMax = std::max(posA(0), posB(0));
            unsigned indexXMin = floor(xMin / cellLength);
            unsigned indexXMax = floor(xMax / cellLength);
            unsigned indexYMin = floor(yMin / cellLength);
            unsigned indexYMax = floor(yMax / cellLength);
            // get cells
            if (abs(m) < 1) {
                while (indexXMin <= indexXMax) {
                    // calculate y value
                    unsigned indexTestT = floor(f0Red + m * indexXMin);
                    unsigned indexTestB = boost::algorithm::clamp(floor(f0Red + m * (indexXMin + 1)),
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
                        unsigned indexTestL = floor((indexYMin - f0Red) / m);
                        unsigned indexTestR = boost::algorithm::clamp(floor(((indexYMin + 1) - f0Red) / m),
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
            const Eigen::Vector3d &pos = iComponent->get_positions()[0];
            const double &a = iComponent->get_parameters()[0];
            const double &b = iComponent->get_parameters()[1];
            // get lowest and highest x and y
            double xMin = pos(0) - a;
            double xMax = pos(0) + a;
            double yMin = pos(1) - b;
            double yMax = pos(1) + b;
            unsigned indexXMin = floor(xMin / cellLength);
            unsigned indexXMax = floor(xMax / cellLength);
            unsigned indexYMin = floor(yMin / cellLength);
            unsigned indexYMax = floor(yMax / cellLength);
            // get grid cells
            if (indexXMin != indexXMax) {
                while (indexXMin <= indexXMax) {
                    double xL = boost::algorithm::clamp(cellLength * indexXMin, xMin, xMax);
                    double xR = boost::algorithm::clamp(cellLength * (indexXMin + 1), xMin, xMax);
                    double yL = sqrt(b * b * (1 - pow((xL - pos(0)) / a, 2)));
                    double yR = sqrt(b * b * (1 - pow((xR - pos(0)) / a, 2)));
                    indexYMin = std::min(floor((pos(1) - yL) / cellLength), floor((pos(1) - yR) / cellLength));
                    indexYMax = std::max(floor((pos(1) + yL) / cellLength), floor((pos(1) + yR) / cellLength));
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
            Eigen::Vector3d &posA = iComponent->get_positions()[0];
            Eigen::Vector3d &posB = iComponent->get_positions()[1];
            // get lowest and largest x and y
            double xMin = std::min(posA(0), posB(0));
            double xMax = std::max(posA(0), posB(0));
            double yMin = std::min(posA(1), posB(1));
            double yMax = std::max(posA(1), posB(1));
            // get grid cells
            unsigned xA = floor(xMin / cellLength);
            unsigned xB = floor(xMax / cellLength);
            unsigned yA = floor(yMin / cellLength);
            unsigned yB = floor(yMax / cellLength);
            while (xMin < xMax) {
                unsigned x = floor(xMin / cellLength);
                unsigned indexA = x * resolution + yA;
                unsigned indexB = x * resolution + yB;
                assignedCells.insert(cells[indexA]);
                assignedCells.insert(cells[indexB]);
                cells[indexA]->add_component(iComponent);
                cells[indexB]->add_component(iComponent);
                xMin += cellLength;
            }
            while (yMin < yMax) {
                unsigned y = floor(yMin / cellLength);
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