/*
 * some self made math functions
 */

#pragma once

#ifndef SRC_MATH_H_
#define SRC_MATH_H_

namespace bmath {
    // checks if number lies in range between min and max with v € [min,max)
    template<typename T>
    bool isInBounds(const T &v, const T &a, const T &b) {
        if (a < b) {
            return !(v < a) && (v < b);
        } else {
            return !(v < b) && (v < a);
        }
    }

    // checks if number lies in range between min and max with v € [min,max]
    template<typename T>
    bool isInBoundsC(const T &v, const T &a, const T &b) {
        if (a < b) {
            return !(v < a) && !(b < v);
        } else {
            return !(v < b) && !(a < v);
        }
    }

    /*
     * return the angle in rads between two vectors
     */

    template<typename T>
    T angleVector2d(
            const T &x1,
            const T &y1,
            const T &x2,
            const T &y2
    ) {
        T _dot = x1 * x2 + y1 * y2;
        T _det = x2 * y1 - x1 * y2;
        return atan2(_det, _dot);
    }

    template<typename T>
    T angleVector2d(
            const T &x1,
            const T &x2
    ) {
        return angleVector2d((double) x1, (double) x2, 1.0d, 0.0d);
    }

    /*
     * checks if the primary element of a pair is larger than the second
     * element of another pair
     */

    template<typename S, typename T>
    bool sortpairbypri(const std::pair<S, T> &a, const std::pair<S, T> &b) {
        return (a.first < b.first);
    };

    /*
     * checks if the second element of a pair is larger than the second
     * element of another pair
     */

    template<typename S, typename T>
    bool sortpairbysec(const std::pair<S, T> &a, const std::pair<S, T> &b) {
        return (a.second < b.second);
    };
};


#endif //SRC_MATH_H_
