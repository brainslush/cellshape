/*
 * settings.h
 *
 *  Created on: May 29, 2017
 *      Author: siegbahn
 */

#pragma once

#ifndef SRC_SETTINGS_H_
#define SRC_SETTINGS_H_

class sSettings {
public:
    double deltaT;
    double &referenceLength;
    unsigned long sideLength;

    sSettings(double &refL) :
            referenceLength(refL) {
    }
    ~sSettings() = default;
};

#endif /* SRC_SETTINGS_H_ */
