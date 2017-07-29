//
// Created by siegbahn on 29.07.17.
//

#ifndef SRC_MATH_H_
#define SRC_MATH_H_

namespace bmath {
    // checks if number lies in range between min and max with v € [min,max)
    template<typename T>
    bool isInBounds(const T &v, const T &min, const T &max) {
        return !(v < min) && (v < max);
    }

    // checks if number lies in range between min and max with v € [min,max]
    template<typename T>
    bool isInBoundsC(const T &v, const T &min, const T &max) {
        return !(v < min) && !(max < v);
    }
};

#endif //SRC_MATH_H_
