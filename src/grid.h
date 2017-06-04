#include "extIncludes.h"
#include "base.h"
#include "gui.h"

#ifndef SRC_GRID_H_
#define SRC_GRID_H_

/*
*  The Grid is supposed to ease up workload for finding near neighbors and to increase overall performance
*  The grid contains cells and each grid cell stores pointers to all components which are located in the gird cell
*/
class grid_border : public base {
public:
    grid_border(double iX1, double iY1, double iX2, double iY2);
    ~grid_border();
    void obtain_visualObjs(std::vector<visual_base*>& iVisualObjs);
protected:
    visual_base* associatedVisualObj;
};

class grid_cell : public base {
public:
    grid_cell(bool*& iShowGrid,bool*& iShowGridOccupation, double iX1, double iY1, double iX2, double iY2);
    ~grid_cell();
    std::set<base*>& get_components();
    void obtain_visualObjs(std::vector<visual_base*>& iVisualObjs);
    bool obtain_intersecting(base* iComponentA, base* iComponentB);
    void update_intersecting();
    void remove_component(base* iComponent);
    void add_component(base* iComponent);
protected:
    bool*& showGrid;
    bool*& showGridOccupation;
    visual_base* associatedVisualObj;
    std::vector<grid_border*> borders;
    std::pair<base*, Eigen::Vector3d> obtain_intersectingCircleLine(base* iRef, base* iCom);
    std::pair<base*, std::pair<Eigen::Vector3d, Eigen::Vector3d>> obtain_intersectingLineLine(base* iRef, base* iCom);
    std::set<base*> components;
    typedef boost::geometry::model::point<
        double,
        2,
        boost::geometry::cs::cartesian
    > point_t;
    boost::geometry::model::box<point_t> boxA;
    boost::geometry::model::box<point_t> boxB;
    boost::geometry::model::ring<point_t> ringA;
    boost::geometry::model::ring<point_t> ringB;
    boost::geometry::model::linestring<point_t> lineA;
    boost::geometry::model::linestring<point_t> lineB;
};

class grid_base {
public:
    //grid_base();
    grid_base(mygui::gui*& iGuiBase,unsigned long long iResolution, double iSideLength);
    ~grid_base();
    void obtain_visualObjs(std::vector<visual_base*>& iVisualObjs);
    void register_component(base* iComponent);
    void unregister_component(base* iComponent);
    void update_component(base* iComponent);
    void update_components();
protected:
    mygui::gui*& guiBase;
    //sSettings& settings;
    unsigned long long resolution;
    double sideLength;
    bool lineSegmentIntersection();
    std::vector<grid_cell*> cells;
    std::set<base*> components;
    mygui::group* guiGroup;
    bool* showGrid;
    bool* showGridOccupation;
};

#endif
