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
    folder = new ofxDatGuiFolder(std::move(iName));
}

group::~group() {
    for (auto it : settings) {
        delete it;
        it = nullptr;
    }
    delete folder;
    folder = nullptr;
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

gui::gui(std::string& iName) {
    datGui = new ofxDatGui(ofxDatGuiAnchor::TOP_RIGHT);
    datGui->addHeader(iName);
    datGui->addFooter();
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
    auto newGroup = new group(std::move(iName));
    groups.insert(newGroup);
    datGui->addFolder(newGroup->get_folder());
    return newGroup;
};

void gui::unregister_group(group *iGroup) {
    groups.erase(iGroup);
    delete iGroup;
    iGroup = nullptr;
}

container::container() = default;

container::~container() {
    for (auto it : guis) {
        delete it;
        it = nullptr;
    }
}

void container::update() {
    for (auto &it : guis) {
        it->update();
    }
}

gui *container::register_gui(std::string iName) {
    auto tmp = new gui(iName);
    guis.insert(tmp);
    return tmp;
}

void container::unregister_gui(gui *iGui) {
    guis.erase(iGui);
    iGui = nullptr;
}
