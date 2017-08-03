//
// Created by siegbahn on 29.07.17.
//

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
};

#endif //SRC_MATH_H_
