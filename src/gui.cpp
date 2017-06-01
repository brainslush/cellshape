/*
 * gui.cpp
 *
 *  Created on: Jun 1, 2017
 *      Author: siegbahn
 */

#include "gui.h"

gui_group::gui_group() {
    name = "";
}
gui_group::gui_group(std::string iName) : name(iName) {

}
gui_group::~gui_group() {
    for(auto& it : settings) {
        delete it;
    }
}
void gui_group::setup() {
    for(auto& it : settings) {
        panel.add(it->setup());
    }
}
void gui_group::draw() {

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
gui_group* gui::register_group() {
    gui_group* newGroup = new gui_group();
    groups.insert(newGroup);
    return newGroup;
};
gui_group* gui::register_group(std::string iName) {
    gui_group* newGroup = new gui_group(iName);
    groups.insert(newGroup);
    return newGroup;
};
