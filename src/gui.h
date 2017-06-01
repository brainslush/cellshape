/*
 * gui.h
 *
 *  Created on: Jun 1, 2017
 *      Author: siegbahn
 */

#include "extIncludes.h"
#include "ofxGui.h"
#include "globalVars.h"

#ifndef SRC_GUI_H_
#define SRC_GUI_H_

using ofxControlTypes = boost::variant<
    ofxButton*,
    ofxToggle*,
    ofxFloatSlider*,
    ofxIntSlider*
>;

template<typename T> class gui_setting {
public:
    gui_setting(std::string iName) : name(iName) {
        control = new ofxButton;
    }
    gui_setting (
        std::string iName,
        T iValue
    ) :
        name(iName),
        value(iValue)
    {
        control = new ofxToggle;
        minValue = false;
        maxValue = false;
    }
    gui_setting(
        std::string iName,
        T iMinValue,
        T iMaxValue,
        T value
    ) :
        name(iName),
        minValue(iMinValue),
        maxValue(iMaxValue)
    {
        if(
            typeid(double) == typeid(value) ||
            typeid(float) == typeid(value)
        ) {
            control = new ofxFloatSlider;
        } else {
            control = new ofxIntSlider;
        }
    }
    virtual ~gui_setting() {
        delete control;
        control == NULL;
    }
    virtual std::string& get_name() {
        return name;
    }
    virtual ofxBaseGui*& get_control() {
        return control;
    }
    virtual T* get_pointer() {
        return &T;
    }
    virtual T& get_value() {
        return value;
    }
    virtual ofxBaseGui* setup() {
        control.ge
    };
private:
    std::string name;
    T value;
    T minValue;
    T maxValue;
    ofxControlTypes control;
};

class gui_group {
public:
    gui_group();
    gui_group(std::string iName);
    virtual ~gui_group();
    virtual void setup();
    virtual void draw();;
    template <typename T,typename ... A>
    T* gui_group::register_setting(std::string iName, A... iArgs) {
        gui_setting<T>* newSetting = new gui_setting<T>(iName, iArgs...);
        settings.insert(newSetting);
        return newSetting->get_pointer();
    }
private:
    std::string name;
    std::set<gui_setting*> settings;
    ofxPanel panel;
};


class gui {
public:
    gui();
    virtual ~gui();
    virtual void setup();
    virtual void draw();
    gui_group* register_group();
    gui_group* register_group(std::string iName);
private:
    std::set<gui_group*> groups;
};

#endif /* SRC_GUI_H_ */
