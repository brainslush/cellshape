//
// Created by brainslush on 14/01/18.
//

#include <eigen3/Eigen/Eigen>
#include "base.h"
#include "bmath.h"

#ifndef CELLFORMATION_LAZYSWEEP_H
#define CELLFORMATION_LAZYSWEEP_H

#define DMIN_ std::numeric_limits<double>::min()

namespace lazySweep {

    /*
     * intersection class which contains a found intersection
     */

    class intersection {
    public:
        intersection(
                const Eigen::Vector3d &iPos,
                base &iObjA,
                base &iObjB
        ) :
                pos(std::move(iPos)),
                objA(iObjA),
                objB(iObjB) {

        };

        ~intersection() = default;

        Eigen::Vector3d pos;
        base &objA;
        base &objB;
    };


    /*
     * line segment class
     */

    class segment {
    public:
        segment(
                unsigned iid,
                base *iObj,
                Eigen::Vector3d &iPosB,
                Eigen::Vector3d &iPosE
        ) :
                id(iid),
                obj(iObj),
                beg(iPosB),
                end(iPosE) {};

        ~segment() = default;

        base *obj;
        unsigned id;
        Eigen::Vector3d &beg;
        Eigen::Vector3d &end;

    protected:
    };

    /*
    * event class which holds all the information of an event
    */

    class event {
    public:
        event(
                segment *iSeg, Eigen::Vector3d iX, unsigned t
        ) :
                seg(iSeg), segSwap(nullptr), x(std::move(iX)), type(t), id(-1) {};

        event(
                segment *iSeg, segment *iSegSwap, Eigen::Vector3d iX, unsigned t
        ) :
                seg(iSeg), segSwap(iSegSwap), x(std::move(iX)), type(t) {};

        ~event() = default;

        segment *seg;
        segment *segSwap;
        Eigen::Vector3d x;
        // 0 - start, 1 - end, 2 - swap
        unsigned type;
        int id;
    };

    /*
     * sort elements by its x-coordinate
     */

    class eventSorterX {
    public:
        bool operator()(const event *a, const event *b) {
            if (a->x[0] < b->x[0]) { return true; }
            if (a->x[0] > b->x[0]) { return false; }
            if (a->type < b->type) { return true; }
            return a->seg->id < b->seg->id;
        }
    };

    /*
     * check if two line segments intersect
     */

    Eigen::Vector3d *intersectingLineLine(segment &segA, segment &segB);

    /*
     * handle intersection
     */

    void checkintersection(
            segment *segA, segment *segB,
            std::set<event *, eventSorterX> &events,
            std::vector<intersection> &intersections
    );

    /*
     * do my lazy sweep
     */

    std::vector<intersection> sweep(const std::set<base *> &iObjects);
}


#endif //CELLFORMATION_LAZYSWEEP_H
