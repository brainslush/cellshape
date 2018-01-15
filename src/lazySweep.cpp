//
// Created by brainslush on 14/01/18.
//

#include "lazySweep.h"

Eigen::Vector3d *lazySweep::intersectingLineLine(segment &segA, segment &segB) {
    if (&segA == &segB) { return nullptr; }
    auto &_posABeg = segA.beg;
    auto &_posAEnd = segA.end;
    auto &_posBBeg = segB.beg;
    auto &_posBEnd = segB.end;

    Eigen::Vector3d _unitDiffRef = (_posAEnd - _posABeg).normalized();
    Eigen::Vector3d _unitDiffCom = (_posBEnd - _posBBeg).normalized();

    auto _rcs = _unitDiffRef[0] * _unitDiffCom[1] - _unitDiffRef[1] * _unitDiffCom[0];

    if (abs(_rcs) >= std::numeric_limits<double>::min()) {
        Eigen::Vector3d _diffPos = _posBBeg - _posABeg;
        double _t = (_diffPos[0] * _unitDiffCom[1] - _diffPos[1] * _unitDiffCom[0]) / _rcs;
        double _u = (_diffPos[0] * _unitDiffRef[1] - _diffPos[1] * _unitDiffRef[0]) / _rcs;
        if (0 <= _t <= 1 && 0 <= _u <= 1) {
            auto _ret = new Eigen::Vector3d(_posABeg + _t * _unitDiffRef);
            auto _a = bmath::isInBoundsC((*_ret)[0], _posABeg[0], _posAEnd[0]);
            auto _b = bmath::isInBoundsC((*_ret)[1], _posABeg[1], _posAEnd[1]);
            auto _c = bmath::isInBoundsC((*_ret)[0], _posBBeg[0], _posBEnd[0]);
            auto _d = bmath::isInBoundsC((*_ret)[1], _posBBeg[1], _posBEnd[1]);
            if (_a && _b && _c && _d) { return _ret; };
        }
        return nullptr;
    }
    return nullptr;
}

void lazySweep::checkintersection(
        segment *segA,
        segment *segB,
        std::set<event *, eventSorterX> &events,
        std::vector<intersection> &intersections
) {
    // check if intersections was checked before
    auto _checked = segA->obj->isIntersectorChecked(segB->obj);
    if (_checked) { return void(); }
    // check for intersection
    auto _intersection = intersectingLineLine(*segA, *segB);
    if (_intersection) {
        intersections.push_back(intersection(*_intersection, *segA->obj, *segB->obj));
        Eigen::Vector3d _posCopy = *_intersection;
        segA->obj->add_intersector(segB->obj, _posCopy);
        segB->obj->add_intersector(segA->obj, _posCopy);
        //events.insert(new event(segA, segB, _posCopy, 2));
        delete _intersection;
        _intersection = nullptr;
    } else {
        segA->obj->add_intersectorChecked(segB->obj);
        segB->obj->add_intersectorChecked(segA->obj);
    }
}

std::vector<lazySweep::intersection> lazySweep::sweep(const std::set<base *> &iObjects) {
    // create return vector
    std::vector<intersection> _intersections;
    // create segments
    std::vector<segment *> _segments;
    //std::multimap<std::pair<double, bool>, segment *, eventsSorter> _events;
    std::set<event *, eventSorterX> _events;
    // insert events sorted by their x position
    for (auto _it : iObjects) {
        if (_it) {
            auto &_posA = _it->get_positions()[0];
            auto &_posB = _it->get_positions()[1];
            auto &_posAx = _posA(0);
            auto &_posBx = _posB(0);
            if ((_posA - _posB).norm() > DMIN_) {
                if (_posAx < _posBx) {
                    _segments.push_back(new segment(_segments.size(), _it, _posA, _posB));
                    _events.insert(new event(_segments.back(), _posA, 0));
                    _events.insert(new event(_segments.back(), _posB, 1));
                } else {
                    _segments.push_back(new segment(_segments.size(), _it, _posB, _posA));
                    _events.insert(new event(_segments.back(), _posB, 0));
                    _events.insert(new event(_segments.back(), _posA, 1));
                }
            }
        } else {
            if (DEBUG_ >= 1) {
                std::cout << "Error (LineSweep) : invalid object added\n";
            }
        }
    }
    while (!_events.empty()) {
        event *_event = *_events.begin();
        auto _isBegining = _event->type;
        auto _segment = _event->seg;
        auto _segmentSwap = _event->segSwap;
        auto _pos = _event->x;
        _events.erase(_events.begin());
        delete _event;
        _event = nullptr;

        // handle start of segment
        if (_isBegining == 0) {
            auto _it = _events.begin();
            while ((*_it)->seg != _segment) {
                if ((*_it)->type == 0) {
                    checkintersection((*_it)->seg, _segment, _events, _intersections);
                }
                _it++;
            }
            _events.erase(_it);
        }
    }

    for (auto _it :_segments) {
        delete _it;
        _it = nullptr;
    }
    if (DEBUG_ >= 1) {
        std::cout << "Info (line sweep) : " << _intersections.size() << "intersections found!\n";
    }
    return _intersections;
}