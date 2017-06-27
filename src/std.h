//
// Created by siegbahn on 27.06.17.
//

#include <utility>

#ifndef SRC_STD_H_
#define SRC_STD_H_

namespace std {
    template<typename T, typename U>
    pair<T, U> operator+(const pair<T, U> &l, const pair<T, U> &r) {
        return {l.first + r.first, l.second + r.second};
    }

    template<typename T, typename U>
    pair<T, U> operator-(const pair<T, U> &l, const pair<T, U> &r) {
        return {l.first - r.first, l.second - r.second};
    }

    template<typename T, typename U>
    pair<T, U> &operator+=(pair<T, U> &l, const pair<T, U> &r) {
        l.first += r.first;
        l.second += r.second;
        return l;
    };

    template<typename T, typename U>
    pair<T, U> operator-=(pair<T, U> &l, const pair<T, U> &r) {
        l.first -= r.first;
        l.second -= r.second;
        return l;
    };
}

#endif //SRC_STD_H
