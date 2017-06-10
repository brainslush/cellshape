#include "ofApp.h"

ofApp::ofApp() :
        Globals(sGlobalVars(new mygui::gui())),
        guiGroup(Globals.guiBase->register_group("General")),
        maxFPS(guiGroup->register_setting<unsigned>("Max FPS", true, 0, 60, 10)) {

    guiGroup->register_action<void()>("Start", [this]() { halt = false; });
    guiGroup->register_action<void()>("Stop", [this]() { halt = true; });
    guiGroup->register_action<void()>("Reset", [this]() {
                                                          halt = true;
                                                          Cell->reset();
                                                          Surface->reset();
                                                          Globals.grid->update_components();
                                                      }
    );
    halt = true;
}

void ofApp::start() {
    halt = false;
}

void ofApp::stop() {
    halt = true;
}

void ofApp::reset() {
    halt = true;
    Cell->reset();
    Surface->reset();
    Globals.grid->update_components();
}

void ofApp::setup() {
    // openframeworks settings
    ofSetFrameRate(10);
    ofBackground(200, 200, 200);
    // set settings
    Globals.settings.deltaT = 1;
    // create grid
    Globals.grid = new grid_base(Globals.guiBase, 100, sideLength);
    // initialize random
    Globals.rndC = new random_container();
    Globals.rndC->set_seed();
    // initialize some global variables
    Globals.time = 0;
    Globals.frameNo = 0;
    // create cell components
    FilamentF = new functor_cell_filamentCreation(Globals); // filament creation functor for cell
    Cell = new cell(Globals, FilamentF);
    Surface = new simple_surface(Globals, sideLength);
}

//--------------------------------------------------------------
void ofApp::update() {
    // calculate scale factor when window is resized
    scale = std::min(ofGetHeight() / (double) sideLength, ofGetWidth() / (double) sideLength);
    // update gui variables
    Globals.guiBase->update();
    // only execute if simulation is not paused
    if (!halt) {
        // update global variables
        Globals.frameNo++;
        Globals.time = Globals.frameNo * Globals.settings.deltaT;
        // update grid
        Globals.grid->update_components();
        // make timestep
        Cell->make_timeStep(Globals.settings.deltaT);
    }
}

//--------------------------------------------------------------
void ofApp::draw() {
    /* obtain visual objects and draw them */
    std::vector<visual_base *> visualObjs;
    Globals.grid->obtain_visualObjs(visualObjs);
    Surface->obtain_visualObjs(visualObjs);
    Cell->obtain_visualObjs(visualObjs);
    for (auto &it : visualObjs) {
        it->draw(scale);
    }
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key) {

}

//--------------------------------------------------------------
void ofApp::keyReleased(int key) {

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y) {

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button) {

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button) {

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button) {

}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y) {

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y) {

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h) {

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg) {

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo) {

}
