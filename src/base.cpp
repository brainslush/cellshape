#include "base.h"

base::base() {
    associatedVisualObj = NULL;
    timeStamp = ofGetFrameNum();
};
base::base(
    std::vector<Eigen::Vector3d> iPositions
):
    positions(iPositions)
{
    associatedVisualObj = NULL;
    timeStamp = ofGetFrameNum();
};
base::base(std::vector<Eigen::Vector3d> iPositions,std::vector<double> iParameters

):
    positions(iPositions),
    parameters(iParameters)
{
    associatedVisualObj = NULL;
    timeStamp = ofGetFrameNum();
};
base::~base() {
    delete associatedVisualObj;
    associatedVisualObj = NULL;
};

void base::set_gridCells(std::set<grid_cell*> iGridCells) { gridCells = iGridCells; }

std::vector<Eigen::Vector3d>& base::get_positions() {return positions;}
std::vector<double>& base::get_parameters() {return parameters;}
std::set<unsigned>& base::get_ignoreIntersect() { return ignoreIntersect; }
std::set<base*>& base::get_intersectorsChecked() { return intersectorsChecked; }
std::set<grid_cell*>& base::get_gridCells() { return gridCells; }
visual_base* base::get_visualObj() {return associatedVisualObj;}
unsigned long long& base::get_timeStamp() {return timeStamp;}

void base::add_intersector(base * iIntersector, Eigen::Vector3d iIntersectorVec) {
    intersectors.push_back(iIntersector);
    intersectorsVectors.push_back(iIntersectorVec);
    intersectorsChecked.insert(iIntersector);
}
void base::add_ignoreIntersect(size_t iIgnore) {
    ignoreIntersect.insert(iIgnore);
}
void base::clear_intersectors() {
    intersectors.clear();
    intersectorsVectors.clear();
    intersectorsChecked.clear();
}
void base::obtain_visualObjs(std::vector<visual_base*>& iVisualObjs) {
    /*do nothing*/
}
void base::update_timeStamp() {
    timeStamp = ofGetFrameNum();
};

/***************************
 * visual_base
 ***************************/

visual_base::visual_base(unsigned iType, base* iComponent) {
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
base& visual_base::get_associatedComponent() {
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
std::vector<Eigen::Vector3d>& visual_base::get_positions() {
    return associatedComponent->get_positions();
}
std::vector<double>& visual_base::get_parameters() {
    return associatedComponent->get_parameters();
}
void visual_base::set_associatedComponent(base* iComponent) {
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
void visual_base::draw(double iScale) {
    /* do nothing */
}

/***************************
 * visual_line
 ***************************/
visual_line::visual_line(base* iComponent) : visual_base(1,iComponent) {
}
visual_line::~visual_line() {
}
void visual_line::draw(double iScale) {
    ofSetColor(get_fillColor());
    ofDrawLine(
        iScale* get_positions()[0](0),
        iScale* get_positions()[0](1),
        iScale* get_positions()[1](0),
        iScale* get_positions()[1](1)
    );
}

/***************************
 * visual_ellipse
 ***************************/
visual_ellipse::visual_ellipse(base* iComponent) : visual_base(2,iComponent) {
}
visual_ellipse::~visual_ellipse() {
}
void visual_ellipse::draw(double iScale) {
    ofSetColor(get_fillColor());
    ofDrawEllipse(
        iScale* get_positions()[0](0),
        iScale* get_positions()[0](1),
        2* iScale* get_parameters()[0],
        2* iScale* get_parameters()[1]
    );
}

/***************************
 * visual_rectangle
 ***************************/
visual_rectangle::visual_rectangle(base* iComponent) : visual_base(3,iComponent) {
}
visual_rectangle::~visual_rectangle() {
}
void visual_rectangle::draw(double iScale) {
    ofSetColor(get_fillColor());
    ofDrawRectangle(
        iScale* get_positions()[0](0),
        iScale* get_positions()[0](1),
        iScale* (get_positions()[2](0) - get_positions()[0](0)),
        iScale* (get_positions()[2](1) - get_positions()[0](1))
    );
}

/***************************
 * visual_triangle
 ***************************/
visual_triangle::visual_triangle(base* iComponent) : visual_base(4,iComponent) {
}
visual_triangle::~visual_triangle() {
}
void visual_triangle::draw(double iScale) {
    ofSetColor(get_fillColor());
    ofDrawTriangle(
        iScale* get_positions()[0](0),
        iScale* get_positions()[0](1),
        iScale* get_positions()[1](0),
        iScale* get_positions()[1](1),
        iScale* get_positions()[2](0),
        iScale* get_positions()[2](1)
    );
}
