#pragma once

#include "base.h"
#include "gui.h"
#include "bmath.h"
#include "ignoreList.h"

#ifndef SRC_GRID_H_
#define SRC_GRID_H_

/*
*  The Grid is supposed to ease up workload for finding near neighbors and to increase overall performance
*  The grid contains cells and each grid cell stores pointers to all components which are located in the gird cell
*/

namespace grid {

    class border : public base {
    public:
        border(double iX1, double iY1, double iX2, double iY2);

        ~border();

        void obtain_visualObjs(std::vector<visual_base *> &iVisualObjs);

    protected:
        visual_base *associatedVisualObj;
    };

    class cell : public base {
    public:
        cell(bool &iShowGrid, bool &iShowGridOccupation, double iX1, double iY1, double iX2, double iY2);

        ~cell();

        std::set<base *> &get_components();

        void obtain_visualObjs(std::vector<visual_base *> &iVisualObjs);

        bool obtain_intersecting(base *iComponentA, base *iComponentB);

        void reset();

        void update_intersecting();

        void remove_component(base *iComponent);

        void add_component(base *iComponent);

    protected:
        bool isOccupied;
        bool &showGrid;
        bool &showGridOccupation;
        visual_base *associatedVisualObj;
        std::vector<border *> borders;

        std::pair<base *, Eigen::Vector3d> obtain_intersectingCircleLine(base *iRef, base *iCom);

        std::pair<base *, std::pair<Eigen::Vector3d, Eigen::Vector3d>>
        obtain_intersectingLineLine(base *iRef, base *iCom);

        std::set<base *> components;
    };

    class container {
    public:
        container(mygui::gui *&iGuiBase, double iSideLength);

        ~container();

        void obtain_visualObjs(std::vector<visual_base *> &iVisualObjs);

        void register_component(base *iComponent);

        void unregister_component(base *iComponent);

        void update_component(base *iComponent);

        void update_components();

        void reset();

    protected:
        void create_cells();

        mygui::gui *&guiBase;
        double sideLength;
        std::vector<cell *> cells;
        std::set<base *> components;
        mygui::group *guiGroup;
        bool &showGrid;
        bool &showGridOccupation;
        unsigned &resolution;
    };
}

#endif
