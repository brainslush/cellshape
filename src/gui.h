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

    using onButtonEvent = std::function<void(ofxDatGuiButtonEvent)>;
    using ofxControlTypes = boost::variant<
            ofxDatGuiLabel *,
            ofxDatGuiToggle *,
            ofxDatGuiSlider *,
            ofxDatGuiColorPicker *>;

    class setting_base {
    public:
        setting_base() = default;

        virtual ~setting_base() = default;

        virtual void update() {};

        virtual void update(bool iGet) {};

    protected:
    };

    template<typename T>
    class setting : public setting_base {
    public:
        setting(
                ofxDatGuiFolder *iFolder,
                std::string &iLabel,
                bool &iUpdatePerFrame,
                T iValue) : updatePerFrame(iUpdatePerFrame),
                            value(iValue) {
            valuePointer = &value;
            if (typeid(bool) == typeid(T)) {
                control = iFolder->addToggle(iLabel, value);
            } else {
                std::stringstream s;
                s << "ERROR wrong type (" << iLabel << ")";
                control = iFolder->addLabel(s.str());
            }
        }

        setting(
                ofxDatGuiFolder *iFolder,
                std::string &iLabel,
                bool &iUpdatePerFrame,
                T iMinValue,
                T iMaxValue,
                T iValue
        ) :
                updatePerFrame(iUpdatePerFrame),
                value(iValue) {
            valuePointer = &value;
            if (std::is_floating_point<T>::value || std::is_integral<T>::value) {
                control = iFolder->addSlider(iLabel, (float) iMinValue, (float) iMaxValue, (double) value);
            } else {
                std::stringstream s;
                s << "ERROR wrong type (" << iLabel << ")";
                control = iFolder->addLabel(s.str());
            }
        }

        setting(
                ofxDatGuiFolder *iFolder,
                std::string &iLabel,
                bool &iUpdatePerFrame,
                T iMinValue,
                T iMaxValue,
                T iValue,
                int iPrecision
        ) :
                updatePerFrame(iUpdatePerFrame),
                value(iValue) {
            valuePointer = &value;
            if (std::is_floating_point<T>::value || std::is_integral<T>::value) {
                control = iFolder->addSlider(iLabel, (float) iMinValue, (float) iMaxValue, (double) value);
                //auto _el = dynamic_cast<ofxDatGuiSlider *>(&control);
                //_el->setPrecision(iPrecision);
            } else {
                std::stringstream s;
                s << "ERROR wrong type (" << iLabel << ")";
                control = iFolder->addLabel(s.str());
            }
        }

        virtual ~setting() = default;

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
        bool updatePerFrame;
        T value;
        T *valuePointer;
        ofxControlTypes control;
    };

    class action_base {
    public:
        action_base() = default;

        virtual ~action_base() = default;

    protected:

    };

    template<typename T>
    class action : public action_base {
    public:
        action(
                ofxDatGuiFolder *iFolder,
                std::string &iLabel,
                std::function<T> iFunction
        ) :
                control(iFolder->addButton(iLabel)),
                function(iFunction) {
            eh = [this](ofxDatGuiButtonEvent e) {
                function();
            };
            control->onButtonEvent(eh);
        };

        virtual ~action() = default;

    protected:
        onButtonEvent eh;
        std::function<T> function;
        ofxDatGuiButton *control;
    };

    class group {
    public:
        group();

        explicit group(std::string iName);

        virtual ~group();

        virtual ofxDatGuiFolder *&get_folder();

        virtual void variableUpdate();

        virtual void forceVariableUpdate();

        template<typename T>
        void register_action(
                std::string iLabel,
                std::function<T> iFunction
        ) {
            iLabel.insert(0, "> ");
            auto newAction = new action<T>(
                    folder,
                    iLabel,
                    iFunction
            );
            actions.insert(newAction);
        };

        template<typename T, typename... A>
        T &register_setting(
                std::string iLabel,
                A... iArgs
        ) {
            iLabel.insert(0, "> ");
            auto newSetting = new setting<T>(
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
            iLabel.insert(0, "> ");
            auto newSetting = new setting<T>(
                    folder,
                    iLabel,
                    iUpdatePerFrame,
                    iArgs...);
            settings.insert(newSetting);
            return newSetting->get_value();
        }

    protected:
        std::set<setting_base *> settings;
        std::set<action_base *> actions;
        std::set<group *> groups;
        ofxDatGuiFolder *folder;
    };

    class gui {
    public:
        explicit gui(std::string &iName);

        virtual ~gui();

        virtual void update();

        virtual group *register_group(std::string iName);

        virtual void unregister_group(group *iGroup);

    protected:
        std::set<group *> groups;
        ofxDatGui *datGui;
    };

    class container {
    public:
        container();

        virtual ~container();

        virtual void update();

        virtual gui *register_gui(std::string iName);

        virtual void unregister_gui(gui *iGui);

    protected:
        std::set<gui *> guis;
    };
};

#endif /* SRC_GUI_H_ */
