/*
 * gui.cpp
 *
 *  Created on: Jun 1, 2017
 *      Author: siegbahn
 */

#include "gui.h"

using namespace mygui;

group::group() {
    folder = new ofxDatGuiFolder("");
}

group::group(std::string iName) {
    folder = new ofxDatGuiFolder(iName);
}

group::~group() {
    for (auto it : settings) {
        delete it;
        it = nullptr;
    }
    delete folder;
    folder = nullptr;
}

group* group::register_group(std::string iName) {
    group *newGroup = new group(iName);
    groups.insert(newGroup);
    return newGroup;
}

ofxDatGuiFolder *&group::get_folder() {
    return folder;
}

void group::variableUpdate() {
    for (auto &it : settings) {
        it->update();
    }
}

void group::forceVariableUpdate() {
    for (auto &it : settings) {
        it->update(true);
    }
}

gui::gui() {
    datGui = new ofxDatGui(ofxDatGuiAnchor::TOP_RIGHT);
}

gui::~gui() {
    for (auto it : groups) {
        delete it;
        it = nullptr;
    }
    delete datGui;
    datGui = nullptr;
}

void gui::update() {
    for (auto &it : groups) {
        it->variableUpdate();
    }
}

group *gui::register_group(std::string iName) {
    group *newGroup = new group(iName);
    groups.insert(newGroup);
    datGui->addFolder(newGroup->get_folder());
    return newGroup;
};

void gui::unregister_group(group *iGroup) {
    groups.erase(iGroup);
    delete iGroup;
    iGroup = nullptr;
}
