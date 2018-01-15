//
// Created by brainslush on 09/01/18.
//

#include "lineSweep.h"

using sweepSet = std::set<lineSweep::singleSweep *, lineSweep::sweepSorter>;

Eigen::Vector3d *lineSweep::intersectingLineLine(segment &segA, segment &segB) {
    if (&segA == &segB) { return nullptr; }
    auto &_posABeg = segA.beg;
    auto &_posAEnd = segA.end;
    auto &_posBBeg = segB.beg;
    auto &_posBEnd = segB.end;

    /*
    if (
            bmath::isInBoundsC(_posBBeg[1], _posABeg[1], _posAEnd[1])
            || bmath::isInBoundsC(_posBEnd[1], _posABeg[1], _posAEnd[1])) {
        return nullptr;
    };
     */
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


void lineSweep::checkintersection(
        segment *segA,
        segment *segB,
        std::set<event *, eventSorter> &events,
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
        events.insert(new event(segA, segB, _posCopy, 2));
        delete _intersection;
        _intersection = nullptr;
    } else {
        segA->obj->add_intersectorChecked(segB->obj);
        segB->obj->add_intersectorChecked(segA->obj);
    }
}

void lineSweep::updateSweep(const sweepSet &sweep) {
    unsigned _i = 0;
    for (auto _it : sweep) {
        _it->seg->sweep = _it;
        _it->id = _i;
        _i++;
    }
};

sweepSet::iterator lineSweep::find(const sweepSet &sweep, segment *seg) {
    auto _it = sweep.begin();
    while (_it != sweep.end()) {
        if ((*_it)->seg == seg) { return _it; };
        _it++;
    }
    return _it;
}

std::pair<sweepSet::iterator, sweepSet::iterator> lineSweep::swap(
        sweepSet &sweep,
        sweepSet::iterator &itA,
        singleSweep *segB,
        const Eigen::Vector3d &pos
) {
    auto _segA = *itA;
    sweep.erase(itA);
    //auto _itB = find(sweep, segB->seg);
    auto _itB = sweep.find(segB->seg->sweep);
    sweep.erase(_itB);
    _segA->seg->useCurr = true;
    segB->seg->useCurr = true;
    _segA->seg->curr = pos;
    segB->seg->curr = pos;
    std::swap(_segA->id, segB->id);
    auto _insA = sweep.insert(_segA);
    auto _insB = sweep.insert(segB);
    updateSweep(sweep);
    if (!_insA.second || !_insB.second) {
        std::cout << "Error (line sweep) : inserting swap failed\n";
    }
    if (std::abs(_segA->id - segB->id) > 1) {
        std::cout << "Error (line sweep) : swaps are no longer neighbours\n";
    }
    return {_insA.first, _insB.first};
};

std::vector<lineSweep::intersection> lineSweep::sweep(const std::set<base *> &iObjects) {
    // create return vector
    std::vector<intersection> _intersections;
    // create segments
    std::vector<segment *> _segments;
    //std::multimap<std::pair<double, bool>, segment *, eventsSorter> _events;
    std::set<event *, eventSorter> _events;
    sweepSet _sweep;
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
        if (DEBUG_ >= 2) {
            for (auto _it : _events) {
                unsigned _c = 0;
                if (_it->type == 2) {
                    _c++;
                }
                if (_c > 0) {
                    std::cout << "Info (line sweep) : " << _c << " swap events found\n";
                }
            }
        }
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
            auto _insert = _sweep.insert(new singleSweep(_segment));
            auto _inserted = _insert.first;

            if (DEBUG_ >= 2 && !_insert.second) {
                std::cout << "error (line sweep) : element wasn't added to the sweep\n";
            }

            updateSweep(_sweep);
            if (_sweep.size() > 1) {
                // check intersection w/ element below
                auto _itnext = _inserted;
                _itnext++;
                if (_itnext != _sweep.end()) {
                    checkintersection((*_itnext)->seg, (*_inserted)->seg, _events, _intersections);
                }
                // check intersection w/ element above
                if (_inserted != _sweep.begin()) {
                    auto _itprev = _inserted;
                    _itprev--;
                    checkintersection((*_itprev)->seg, (*_inserted)->seg, _events, _intersections);
                }
            }
            // handle end of segment
        } else if (_isBegining == 1) {
            //auto _find = find(_sweep,_segment);
            auto _find = _sweep.find(_segment->sweep);

            if (_find != _sweep.end()) {
                // delete sweep
                auto _delSeg = *_find;
                delete _delSeg;
                _delSeg = nullptr;
                auto _last = _sweep.erase(_find);
                updateSweep(_sweep);
                if (_sweep.size() > 1 && _last != _sweep.end()) {
                    // check intersection of new contacting elements
                    auto _itnext = _last;
                    _itnext++;
                    if (_itnext != _sweep.end()) {
                        if (_last != _sweep.begin()) {
                            // need check for end
                            auto _itprev = _last;
                            _itprev--;
                            checkintersection((*_itprev)->seg, (*_last)->seg, _events, _intersections);
                        }
                    }
                }
            } else {
                std::cout << "Error (sweep line) :  Couldn't find element to remove!\n";
            }
        } else if (_isBegining == 2) {
            // find crossings
            auto _findSeg = _sweep.find(_segment->sweep);
            auto _findSegPrev = _findSeg;
            auto _findSegNext = _findSeg;
            _findSegNext++;
            _findSegPrev--;
            if ((*_findSegPrev)->seg == _segmentSwap) {
                auto _ret = swap(_sweep, _findSeg, *_findSegPrev, _pos);
                auto _prev = _ret.first;
                auto _next = _ret.second;
                if (_prev != _sweep.begin()) {
                    _prev--;
                    checkintersection((*_prev)->seg, (*_ret.first)->seg, _events, _intersections);
                }
                _next++;
                if (_next != _sweep.end()) {
                    checkintersection((*_next)->seg, (*_ret.second)->seg, _events, _intersections);
                }
            } else if ((*_findSegNext)->seg == _segmentSwap) {
                auto _ret = swap(_sweep, _findSeg, *_findSegNext, _pos);
                auto _prev = _ret.second;
                auto _next = _ret.first;
                if (_prev != _sweep.begin()) {
                    _prev--;
                    checkintersection((*_prev)->seg, (*_ret.second)->seg, _events, _intersections);
                }
                _next++;
                if (_next != _sweep.end()) {
                    checkintersection((*_next)->seg, (*_ret.first)->seg, _events, _intersections);
                }
            } else {
                //std::cout << "Error (sweep line) : swap partners are not neighbours\n";
                auto _findSwap = _sweep.find(_segmentSwap->sweep);
                auto _ret = checkRange(_sweep, _events, _intersections, _findSeg, _findSwap, _pos);
                auto _itA = _ret.first;
                auto _itB = _ret.second;
                if ((*_itA)->id < (*_itB)->id) {
                    if (_itA != _sweep.begin()) {
                        auto _prev = _itA;
                        _prev--;
                        checkintersection((*_prev)->seg, (*_itA)->seg, _events, _intersections);
                    };
                    auto _next = _itB;
                    _next++;
                    if (_next != _sweep.end()) {
                        checkintersection((*_next)->seg, (*_itB)->seg, _events, _intersections);
                    }
                } else {
                    if (_itB != _sweep.begin()) {
                        auto _prev = _itB;
                        _prev--;
                        checkintersection((*_prev)->seg, (*_itB)->seg, _events, _intersections);
                    };
                    auto _next = _itA;
                    _next++;
                    if (_next != _sweep.end()) {
                        checkintersection((*_next)->seg, (*_itA)->seg, _events, _intersections);
                    }
                }
            }
        }
    }
    for (auto _it : _segments) {
        delete _it;
        _it = nullptr;
    }
    if (DEBUG_ >= 1) {
        std::cout << "Info (line sweep) : " << _intersections.size() << "intersections found!\n";
    }
    return _intersections;
}

void lineSweep::runRange(
        const sweepSet &sweep,
        std::set<event *, eventSorter> &events,
        std::vector<intersection> &intersections,
        bool dir,
        sweepSet::iterator &itRef,
        Eigen::Vector3d &pos
) {
    auto _seg = (*itRef)->seg;
    // create back iterator
    auto _back = sweep.end();
    _back--;
    // let's do some checks
    if (!dir && itRef == sweep.begin()) { return void(); }
    if (dir && itRef == _back) { return void(); }
    // run through range
    auto _it = itRef;
    while (_it != sweep.begin() && _it != _back) {
        if (dir) {
            move(_it, dir);
            if ((*_it)->seg->rpos()[1] < pos[1]) {
                checkintersection((*_it)->seg, _seg, events, intersections);
            } else {
                _it = _back;
            }
        } else {
            move(_it, dir);
            if ((*_it)->seg->rpos()[1] > pos[1]) {
                checkintersection((*_it)->seg, _seg, events, intersections);
            } else {
                _it = sweep.begin();
            }
        }

    }
}


std::pair<sweepSet::iterator, sweepSet::iterator> lineSweep::checkRange(
        sweepSet &sweep,
        std::set<event *, eventSorter> &events,
        std::vector<intersection> &intersections,
        sweepSet::iterator &itA,
        sweepSet::iterator &itB,
        Eigen::Vector3d &pos
) {
    auto _range = std::abs((*itA)->id - (*itB)->id);

    bool _dirA = pos[1] > (*itA)->seg->rpos()[1];
    bool _dirB = pos[1] > (*itB)->seg->rpos()[1];

    runRange(sweep, events, intersections, _dirA, itA, pos);
    runRange(sweep, events, intersections, _dirB, itB, pos);

    return swap(sweep, itA, (*itB), pos);
}

void lineSweep::move(sweepSet::iterator &it, bool dir) {
    dir ? it++ : it--;
};
