#pragma once

#include <set>
#include <vector>
#include <eigen3/Eigen/Eigen>
#include "ofMain.h"

#ifndef SRC_BASE_H_
#define SRC_BASE_H_

#define DEBUG_ 2

/*
 * forward declarations
 */

namespace grid {
    class cell;
}

class visual_base;

/*
 * base class for all cell components
 */

class base {
public:
    base();

    explicit base(std::vector<Eigen::Vector3d> iPositions);

    base(std::vector<Eigen::Vector3d> iPositions, std::vector<double> iParameters);

    virtual ~base();

    virtual void set_gridCells(const std::set<grid::cell *> &iGridCells);

    virtual std::vector<Eigen::Vector3d> &get_positions();

    virtual std::vector<double> &get_parameters();

    virtual std::set<base *> &get_intersectorsChecked();

    virtual std::set<grid::cell *> &get_gridCells();

    virtual std::set<std::pair<base *, Eigen::Vector3d *>> &get_intersectors();

    virtual visual_base *get_visualObj();

    virtual std::size_t &get_typeHash();

    virtual bool isIntersectorChecked(base *iRef);

    virtual void add_intersector(base *iIntersector, Eigen::Vector3d iIntersectorVec);

    virtual void add_intersectorChecked(base *iChecked);

    virtual void clear_intersectors();

    virtual void clear_intersectorsChecked();

    virtual void obtain_visualObjs(std::vector<visual_base *> &iVisualObjs);

protected:
    std::vector<Eigen::Vector3d> positions; // position of object
    std::vector<double> parameters; // additional parameters of the object
    std::vector<Eigen::Vector3d> intersectionVectors; // save the actual
    std::set<std::pair<base *, Eigen::Vector3d *>> intersectors; // list of objects which intersect with the object
    std::set<base *> intersectorsChecked; // list of intersectors which are already checked
    std::set<grid::cell *> gridCells; // gridcells in which object lies
    visual_base *associatedVisualObj; // assignes a visual object
    unsigned long long timeStamp; // timestamp is relevant for variableUpdate features
    std::size_t typeHash;
};

/*
 *  Those are all accessible visual elements which can be used to represent the components.
 *  The name might be misleading as the visual element also determines the physical form of the component.
 */

class visual_base {
public:
    visual_base(unsigned iType, base *iComponent);

    virtual ~visual_base();

    virtual ofFloatColor &get_color();

    virtual ofFloatColor &get_fillColor();

    virtual unsigned &get_type();

    virtual std::vector<Eigen::Vector3d> &get_positions();

    virtual std::vector<double> &get_parameters();

    virtual void set_color(double iRed, double iGreen, double iBlue, double iAlpha);

    virtual void set_color(double iRed, double iGreen, double iBlue);

    virtual void set_fillColor(double iRed, double iGreen, double iBlue, double iAlpha);

    virtual void set_fillColor(double iRed, double iGreen, double iBlue);

    virtual void draw(double iScale);

protected:
    base *associatedComponent;
    ofFloatColor color;
    ofFloatColor fillColor;
    unsigned type;
};

class visual_line : public visual_base {
public:
    explicit visual_line(base *iComponent);

    virtual ~visual_line();

    virtual void draw(double iScale);

protected:
};

class visual_ellipse : public visual_base {
public:
    explicit visual_ellipse(base *iComponent);

    virtual ~visual_ellipse();

    virtual void draw(double iScale);

protected:
};

class visual_arcCircle : public visual_base {
public:
    explicit visual_arcCircle(base *iComponent);

    virtual ~visual_arcCircle();

    virtual void draw(double iScale);

protected:
};

class visual_hyperbola : public visual_base {
public:
    explicit visual_hyperbola(base *iComponent);

    virtual ~visual_hyperbola();

    virtual void draw(double iScale);
};

class visual_rectangle : public visual_base {
public:
    explicit visual_rectangle(base *iComponent);

    virtual ~visual_rectangle();

    virtual void draw(double iScale);

protected:
};

class visual_triangle : public visual_base {
public:
    explicit visual_triangle(base *iComponent);

    ~visual_triangle();

    virtual void draw(double iScale);

protected:
};

#endif
