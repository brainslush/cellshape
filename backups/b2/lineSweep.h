//
// Created by brainslush on 09/01/18.
//

#pragma once

#include <eigen3/Eigen/Eigen>
#include "../../src/base.h"
#include "../../src/bmath.h"

#ifndef CELLFORMATION_LINESWEEP_H
#define CELLFORMATION_LINESWEEP_H

#define DMIN_ std::numeric_limits<double>::min()
/*
namespace lineSweep {

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

    class singleSweep;

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
                end(iPosE),
                curr(Eigen::Vector3d(0, 0, 0)),
                useCurr(false),
                sweep(nullptr) {
            Eigen::Vector3d _diff = (end - beg).normalized();
            m = bmath::angleVector2d(_diff[0], _diff[1]);
            if (m > M_PI / 2) {
                m -= M_PI_2;
            }
        };

        ~segment() = default;

        Eigen::Vector3d &rpos() {
            if (useCurr) {
                return curr;
            } else {
                return beg;
            }
        }

        bool operator<(segment &A) {
            return rpos()[1] < A.rpos()[1];
        }

        base *obj;
        unsigned id;
        Eigen::Vector3d &beg;
        Eigen::Vector3d &end;
        Eigen::Vector3d curr;
        bool useCurr;
        double m;
        singleSweep *sweep;

    protected:
    };

    /*
     * pseudo class for
     */

    class singleSweep {
    public:
        singleSweep(segment *iSeg) :
                seg(iSeg),
                id(-1) {};
        segment *seg;
        d
    };

    /*
     * event class which holds all the infromation of an event
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
            if (a->x[0] < b->x[0] - DMIN_) { return true; }
            if (a->x[0] > b->x[0] + DMIN_) { return false; }
            if (a->type == 0) { return true; }
            return false;
        }
    };

    /*
     * Sort elements by its y-coordinate for sweep
     */

    class eventSorterY {
    public:
        bool operator()(const event *a, const event *b) {
            double _posAY = a->seg->useCurr ? a->seg->curr[1] : a->seg->beg[1];
            double _posBY = b->seg->useCurr ? b->seg->curr[1] : b->seg->beg[1];
            if ()
            return a->id < b->id;
        }
    };

    /*
     * typedef
     */

    using sweepSet = std::set<singleSweep *, eventSorterY>;

    /*
     * check if two line segments intersect
     */

    Eigen::Vector3d *intersectingLineLine(segment &segA, segment &segB);

    /*
     * handle intersection
     */

    void checkintersection(
            segment *segA, segment *segB,
            std::set<event *, eventSorter> &events,
            std::vector<intersection> &intersections
    );

    /*
     * update the sweep
     */

    void updateSweep(const sweepSet &sweep);

    /*
     * find via segment adress
     */

    sweepSet::iterator find(const sweepSet &sweep, segment *seg);

    /*
     * up and down
     */

    void move(sweepSet::iterator &it, bool dir);

    /*
     * check range
     */

    void runRange(
            const sweepSet &sweep,
            std::set<event *, eventSorter> &events,
            std::vector<intersection> &intersections,
            bool dir,
            sweepSet::iterator &itRef,
            Eigen::Vector3d &pos
    );

    std::pair<sweepSet::iterator, sweepSet::iterator> checkRange(
            sweepSet &sweep,
            std::set<event *, eventSorter> &events,
            std::vector<intersection> &intersections,
            sweepSet::iterator &itA,
            sweepSet::iterator &itB,
            Eigen::Vector3d &pos
    );

    /*
     * do swap
     */

    std::pair<sweepSet::iterator, sweepSet::iterator> swap(
            sweepSet &sweep,
            sweepSet::iterator &itA,
            singleSweep *segB,
            const Eigen::Vector3d &pos
    );

    /*
     * do a line sweep
     */

    std::vector<intersection> sweep(const std::set<base *> &iObjects);
}

*/

#endif //CELLFORMATION_LINESWEEP_H
