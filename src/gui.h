/*
 * gui.h
 *
 *  Created on: Jun 1, 2017
 *      Author: siegbahn
 */

#pragma once

#include "ofxDatGui.h"
#include <set>
#include <typeinfo>
#include <type_traits>
#include <functional>
#include <boost/variant.hpp>
#include <boost/algorithm/clamp.hpp>

#ifndef SRC_GUI_H_
#define SRC_GUI_H_

namespace mygui {
    using ofxControlTypes = boost::variant<
            ofxDatGuiLabel *,
            ofxDatGuiButton *,
            ofxDatGuiToggle *,
            ofxDatGuiSlider *,
            ofxDatGuiColorPicker *>;

    class setting_base {
    public:
        setting_base() {};

        virtual ~setting_base() {};

        virtual void update() {};

    protected:
    };

    template<typename T>
    class setting : public setting_base {
    public:
        setting(
                ofxDatGuiFolder *iFolder,
                std::string &iLabel,
                T iValue
        ) : label(iLabel),
            value(iValue) {
            valuePointer = &value;
            if (std::is_function<T>::value) {
                control = iFolder->addButton(label);
            } else {
                std::stringstream s;
                s << "ERROR wrong type (" << label << ")";
                control = iFolder->addLabel(s.str());
            }
        }

        setting(
                ofxDatGuiFolder *iFolder,
                std::string &iLabel,
                bool &iUpdatePerFrame,
                T iValue) : label(iLabel),
                            updatePerFrame(iUpdatePerFrame),
                            value(iValue) {
            valuePointer = &value;
            if (typeid(bool) == typeid(T)) {
                control = iFolder->addToggle(label, value);
            } else {
                std::stringstream s;
                s << "ERROR wrong type (" << label << ")";
                control = iFolder->addLabel(s.str());
            }
        }

        setting(
                ofxDatGuiFolder *iFolder,
                std::string &iLabel,
                bool &iUpdatePerFrame,
                T iMinValue,
                T iMaxValue,
                T iValue) : label(iLabel),
                            updatePerFrame(iUpdatePerFrame),
                            value(iValue) {
            valuePointer = &value;
            if (std::is_floating_point<T>::value || std::is_integral<T>::value) {
                control = iFolder->addSlider(label, (float) iMinValue, (float) iMaxValue, (double) value);
            } else {
                std::stringstream s;
                s << "ERROR wrong type (" << label << ")";
                control = iFolder->addLabel(s.str());
            }
        }

        virtual ~setting() {
        }

        virtual T *&get_pointer() {
            return valuePointer;
        }

        virtual T &get_value() {
            update(true);
            return value;
        }

        virtual void update(bool iGet) {
            if (updatePerFrame || iGet) {
                if (control.type() == typeid(ofxDatGuiToggle *)) {
                    ofxDatGuiToggle *t = boost::get<ofxDatGuiToggle *>(control);
                    value = t->getChecked();
                } else if (control.type() == typeid(ofxDatGuiSlider *)) {
                    ofxDatGuiSlider *t = boost::get<ofxDatGuiSlider *>(control);
                    value = t->getValue();
                }
            }
        };

        virtual void update() {
            update(false);
        };

    protected:
        std::string label;
        bool updatePerFrame;
        T value;
        T *valuePointer;
        ofxControlTypes control;
    };

    class group {
    public:
        group();

        group(std::string iName);

        virtual ~group();

        virtual ofxDatGuiFolder *&get_folder();

        virtual void update();

        template<typename T, typename... A>
        T &register_setting(
                std::string iLabel,
                A... iArgs
        ) {
            setting<T> *newSetting = new setting<T>(
                    folder,
                    iLabel,
                    iArgs...);
            settings.insert(newSetting);
            return newSetting->get_value();
        }

        template<typename T, typename... A>
        T &register_setting(
                std::string iLabel,
                bool iUpdatePerFrame,
                A... iArgs
        ) {
            setting<T> *newSetting = new setting<T>(
                    folder,
                    iLabel,
                    iUpdatePerFrame,
                    iArgs...);
            settings.insert(newSetting);
            return newSetting->get_value();
        }

    protected:
        std::set<setting_base *> settings;
        ofxDatGuiFolder *folder;
    };

    class gui {
    public:
        gui();

        virtual ~gui();

        virtual void update();

        virtual group *register_group(std::string iName);

        virtual void unregister_group(group *iGroup);

    protected:
        std::set<group *> groups;
        ofxDatGui *datGui;
    };
}

#endif /* SRC_GUI_H_ */
