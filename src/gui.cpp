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
        it = NULL;
    }
    delete folder;
    folder = NULL;
}

ofxDatGuiFolder *&group::get_folder() {
    return folder;
}

void group::update() {
    for (auto &it : settings) {
        it->update();
    }
}


gui::gui() {
    datGui = new ofxDatGui(ofxDatGuiAnchor::TOP_LEFT);
}

gui::~gui() {
    for (auto it : groups) {
        delete it;
        it = NULL;
    }
    delete datGui;
    datGui = NULL;
}

void gui::update() {
    for (auto &it : groups) {
        it->update();
    }
}

group *gui::register_group(std::string iName) {
    group *newGroup = new group(iName);
    groups.insert(newGroup);
    datGui->addFolder(newGroup->get_folder());
    return newGroup;
};

void gui::unregister_group(group *iGroup) {
    delete iGroup;
    groups.erase(iGroup);
}
