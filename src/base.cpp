#include "base.h"


base::base() :
        associatedVisualObj(nullptr),
        timeStamp(ofGetFrameNum()),
        typeHash(0) {}

base::base(
        std::vector<Eigen::Vector3d> iPositions
) :
        associatedVisualObj(nullptr),
        timeStamp(ofGetFrameNum()),
        typeHash(0),
        positions(std::move(iPositions)) {}

base::base(
        std::vector<Eigen::Vector3d> iPositions,
        std::vector<double> iParameters
) :
        associatedVisualObj(nullptr),
        timeStamp(ofGetFrameNum()),
        typeHash(0),
        positions(std::move(iPositions)),
        parameters(std::move(iParameters)) {}

base::~base() {
    delete associatedVisualObj;
    associatedVisualObj = nullptr;
}

void base::set_gridCells(const std::set<grid::cell *> &iGridCells) { gridCells = std::move(iGridCells); }

std::vector<Eigen::Vector3d> &base::get_positions() { return positions; }

std::vector<double> &base::get_parameters() { return parameters; }

std::set<base *> &base::get_intersectorsChecked() { return intersectorsChecked; }

std::set<grid::cell *> &base::get_gridCells() { return gridCells; }

std::set<std::pair<base *, Eigen::Vector3d *>> &base::get_intersectors() { return intersectors; };

visual_base *base::get_visualObj() { return associatedVisualObj; }

bool base::isIntersectorChecked(base *iRef) {
    return intersectorsChecked.find(iRef) != intersectorsChecked.end();
};

void base::add_intersector(base *iIntersector, Eigen::Vector3d iIntersectorVec) {
    intersectionVectors.push_back(std::move(iIntersectorVec));
    intersectors.insert(std::make_pair(iIntersector, &*intersectionVectors.end()));
    intersectorsChecked.insert(iIntersector);
}

void base::clear_intersectors() {
    intersectors.clear();
    intersectionVectors.clear();
    clear_intersectorsChecked();
}

void base::obtain_visualObjs(std::vector<visual_base *> &iVisualObjs) {
    /*do nothing*/
}

std::size_t &base::get_typeHash() {
    if (typeHash == 0) {
        typeHash = typeid(*this).hash_code();
    }
    return typeHash;
}

void base::add_intersectorChecked(base *iChecked) {
    intersectorsChecked.insert(iChecked);
}

void base::clear_intersectorsChecked() {
    intersectorsChecked.clear();
}

/***************************
 * visual_base
 ***************************/

visual_base::visual_base(unsigned iType, base *iComponent) {
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

visual_base::~visual_base() = default;

ofFloatColor &visual_base::get_color() {
    return color;
}

ofFloatColor &visual_base::get_fillColor() {
    return fillColor;
}

unsigned &visual_base::get_type() {
    return type;
}

std::vector<Eigen::Vector3d> &visual_base::get_positions() {
    return associatedComponent->get_positions();
}

std::vector<double> &visual_base::get_parameters() {
    return associatedComponent->get_parameters();
}

void visual_base::set_color(double iRed, double iGreen, double iBlue, double iAlpha) {
    color.r = (float) iRed;
    color.g = (float) iGreen;
    color.b = (float) iBlue;
    color.a = (float) iAlpha;
}

void visual_base::set_color(double iRed, double iGreen, double iBlue) {
    color.r = (float) iRed;
    color.g = (float) iGreen;
    color.b = (float) iBlue;
    color.a = 1.0;
}

void visual_base::set_fillColor(double iRed, double iGreen, double iBlue, double iAlpha) {
    fillColor.r = (float) iRed;
    fillColor.g = (float) iGreen;
    fillColor.b = (float) iBlue;
    fillColor.a = (float) iAlpha;
}

void visual_base::set_fillColor(double iRed, double iGreen, double iBlue) {
    fillColor.r = (float) iRed;
    fillColor.g = (float) iGreen;
    fillColor.b = (float) iBlue;
    fillColor.a = 1.0;
}

void visual_base::draw(double iScale) {
    /* do nothing */
}

/***************************
 * visual_line
 ***************************/
visual_line::visual_line(base *iComponent) : visual_base(1, iComponent) {
}

visual_line::~visual_line() = default;

void visual_line::draw(double iScale) {
    ofSetColor(get_fillColor());
    ofDrawLine(
            (float) (iScale * get_positions()[0](0)),
            (float) (iScale * get_positions()[0](1)),
            (float) (iScale * get_positions()[1](0)),
            (float) (iScale * get_positions()[1](1))
    );
}

/***************************
 * visual_ellipse
 ***************************/
visual_ellipse::visual_ellipse(base *iComponent) : visual_base(2, iComponent) {
}

visual_ellipse::~visual_ellipse() = default;

void visual_ellipse::draw(double iScale) {
    ofSetColor(get_fillColor());
    ofDrawEllipse(
            (float) (iScale * get_positions()[0](0)),
            (float) (iScale * get_positions()[0](1)),
            (float) (2 * iScale * get_parameters()[0]),
            (float) (2 * iScale * get_parameters()[1])
    );
}

/***************************
 * visual_rectangle
 ***************************/
visual_rectangle::visual_rectangle(base *iComponent) : visual_base(3, iComponent) {
}

visual_rectangle::~visual_rectangle() = default;

void visual_rectangle::draw(double iScale) {
    ofSetColor(get_fillColor());
    ofDrawRectangle(
            (float) (iScale * get_positions()[0](0)),
            (float) (iScale * get_positions()[0](1)),
            (float) (iScale * (get_positions()[2](0) - get_positions()[0](0))),
            (float) (iScale * (get_positions()[2](1) - get_positions()[0](1)))
    );
}

/***************************
 * visual_triangle
 ***************************/
visual_triangle::visual_triangle(base *iComponent) : visual_base(4, iComponent) {
}

visual_triangle::~visual_triangle() = default;

void visual_triangle::draw(double iScale) {
    ofSetColor(get_fillColor());
    ofDrawTriangle(
            (float) (iScale * get_positions()[0](0)),
            (float) (iScale * get_positions()[0](1)),
            (float) (iScale * get_positions()[1](0)),
            (float) (iScale * get_positions()[1](1)),
            (float) (iScale * get_positions()[2](0)),
            (float) (iScale * get_positions()[2](1))
    );
}

/*
 * visual_acrCircle
 */

visual_arcCircle::visual_arcCircle(base *iComponent) : visual_base(5, iComponent) {

}

visual_arcCircle::~visual_arcCircle() = default;

void visual_arcCircle::draw(double iScale) {
    double &_x = get_positions()[0](0);
    double &_y = get_positions()[0](1);
    double &_R = get_parameters()[0];
    double &_angleB = get_parameters()[1];
    double &_angleE = get_parameters()[2];

    auto _step = 2 * M_PI / 360;
    double _xp = _x + _R * cos(_angleB);
    double _yp = _y + _R * sin(_angleB);
    ofPoint _pos((float) (iScale * _xp), (float) (iScale * _yp));
    ofPolyline _curve({_pos});
    double _angle1 = _angleB;
    double _angle2 = _angleE;
    while (_angle1 > _angle2) {
        _angle2 += 2 * M_PI;
    }
    while (_angle1 + _step <= _angle2) {
        _angle1 += _step;
        _xp = _x + _R * cos(_angle1);
        _yp = _y + _R * sin(_angle1);
        _curve.lineTo((float) (iScale * _xp), (float) (iScale * _yp));
    }
    _xp = _x + _R * cos(_angleE);
    _yp = _y + _R * sin(_angleE);
    _curve.lineTo((float) (iScale * _xp), (float) (iScale * _yp));
    ofSetColor(get_fillColor());
    _curve.draw();
}

visual_hyperbola::visual_hyperbola(base *iComponent) : visual_base(6, iComponent) {

}

visual_hyperbola::~visual_hyperbola() {

}

void visual_hyperbola::draw(double iScale) {
    auto &_pos = get_positions()[0];
    auto &_d = get_parameters()[0];
    auto &_a = get_parameters()[1];
    auto &_b = get_parameters()[2];
    auto &_t1 = get_parameters()[3];
    auto &_t2 = get_parameters()[4];
    auto &_dir = get_parameters()[5];
    auto _xp = _a * cosh(_t1) + _d;
    auto _yp = _b * sinh(_t1);

    auto _step = abs(_t1 - _t2) / 20.0d;

    ofPoint _startpos((float) (iScale * _xp), (float) (iScale * _yp));
    ofPath _curve = ofPath();
    _curve.lineTo(_startpos);

    while (_t1 + _step <= _t2) {
        _t1 += _step;
        _xp = _a * cosh(_t1) + _d;
        _yp = _b * sinh(_t1);
        _curve.lineTo((float) (iScale * _xp), (float) (iScale * _yp));
    }
    _curve.rotate((float)(360 * _dir / M_PI_2), ofPoint(0,0,1));
    _curve.translate(ofPoint(_pos(0) * iScale, _pos(1) * iScale));
    //ofSetColor(get_fillColor());
    _curve.setStrokeColor(get_fillColor());
    _curve.setStrokeWidth(1);
    _curve.setFilled(false);
    _curve.draw();
}
