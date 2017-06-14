#include "ofApp.h"

ofApp::ofApp() {
    Globals.guiBase = new mygui::gui();
    guiGroup = Globals.guiBase->register_group("General");
    //maxFPS =guiGroup->register_setting<unsigned>("Max FPS", true, 0, 60, 10));
    guiGroup->register_action<void()>("Start", [this]() { halt = false; });
    guiGroup->register_action<void()>("Stop", [this]() { halt = true; });
    guiGroup->register_action<void()>("Reset", [this]() {
                                          halt = true;
                                          Cell->reset();
                                          Surface->reset();
                                          Globals.grid->reset();
                                          Globals.grid->update_components();
                                      }
    );
    halt = true;
}

ofApp::~ofApp() {
    delete guiGroup;
    guiGroup = nullptr;
    delete FilamentF;
    FilamentF = nullptr;
    delete Cell;
    Cell = nullptr;
    delete Surface;
    Surface = nullptr;
}

void ofApp::setup() {
    // openframeworks settings
    ofSetFrameRate(10);
    ofBackground(50, 50, 50);
    // set settings
    Globals.settings.deltaT = 1;
    Globals.settings.sideLength = 500;
    // create grid
    Globals.grid = new grid::container(Globals.guiBase, Globals.settings.sideLength);
    // initialize random
    Globals.rndC = new random_container();
    Globals.rndC->set_seed();
    // initialize some global variables
    Globals.time = 0;
    Globals.frameNo = 0;
    // create cell components
    Surface = new simple_surface(Globals, Globals.settings.sideLength);
    FilamentF = new functor_cell_filamentCreation(Globals); // filament creation functor for cell
    Cell = new cell(Globals, FilamentF);
}

//--------------------------------------------------------------
void ofApp::update() {
    // calculate scale factor when window is resized
    scale = std::min(ofGetHeight() / (double) Globals.settings.sideLength,
                     ofGetWidth() / (double) Globals.settings.sideLength);
    // variableUpdate gui variables
    Globals.guiBase->update();
    // only execute if simulation is not paused
    if (!halt) {
        // variableUpdate global variables
        Globals.frameNo++;
        Globals.time = Globals.frameNo * Globals.settings.deltaT;
        // variableUpdate grid
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
