/*
 * gui.cpp
 *
 *  Created on: Jun 1, 2017
 *      Author: siegbahn
 */

#include "gui.h"

using namespace mygui;

group::group() {
    name = "";
}
group::group(std::string iName) : name(iName) {

}
group::~group() {
    for(auto& it : settings) {
        delete it;
    }
}
void group::setup() {
    for(auto& it : settings) {
        panel.add(it->setup());
    }
}
void group::draw() {
    panel.draw();
}
void group::update() {
    for(auto& it : settings) {
        it->update();
    }
}



gui::gui() {

}
gui::~gui() {
    for(auto& it : groups) {
        delete it;
    }
}
void gui::setup() {
    for(auto& it : groups) {
        it->setup();
    }
}
void gui::draw() {
    for(auto& it : groups) {
        it->draw();
    }
}
void gui::update() {
    for(auto& it : groups) {
        it->update();
    }
}
group* gui::register_group() {
    group* newGroup = new group();
    groups.insert(newGroup);
    return newGroup;
};
group* gui::register_group(std::string iName) {
    group* newGroup = new group(iName);
    groups.insert(newGroup);
    return newGroup;
};
void gui::unregister_group(group* iGroup) {
    delete iGroup;
    groups.erase(iGroup);
}
