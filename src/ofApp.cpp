#include "ofApp.h"

ofApp::ofApp() {
    globals.guiC = new mygui::container();
    globals.guiMain = globals.guiC->register_gui("General");
    mygui::group *guiGroup = globals.guiMain->register_group("Control");
    //maxFPS =guiGroup->register_setting<unsigned>("Max FPS", true, 0, 60, 10));
    guiGroup->register_action<void()>("Start", [this]() { halt = false; });
    guiGroup->register_action<void()>("Stop", [this]() { halt = true; });
    guiGroup->register_action<void()>("Reset", [this]() {
                                          halt = true;
                                          ccell->reset();
                                          surface->reset();
                                          globals.grid->reset();
                                          globals.grid->update_components();
                                      }
    );
    halt = true;
    // add classes to the registrar
    registrar::registerType<base, components_base>();
    registrar::registerType<components_base, cell_base>();
    registrar::registerType<cell_base, cell>();
    registrar::registerType<components_base, cellcomponents_base>();
    registrar::registerType<cellcomponents_base, crosslinker_base>();
    registrar::registerType<cellcomponents_base, filament_base>();
    registrar::registerType<filament_base, actin>();
    registrar::registerType<cellcomponents_base, volume_base>();
    registrar::registerType<cellcomponents_base, membrane_part_base>();
    registrar::registerType<components_base, matrixcomponents_base>();
    registrar::registerType<matrixcomponents_base, fac_base>();
    registrar::registerType<matrixcomponents_base, surface_border_base>();
    registrar::registerType<matrixcomponents_base, surface_base>();
    registrar::registerType<fac_base, fac>();
    registrar::registerType<membrane_part_base, membrane_part>();
    registrar::registerType<cellcomponents_base,membrane_container>();
    // add self ignore rules
    ignore::addRule<membrane_part_base, membrane_part_base>();
    ignore::addRule<fac_base, fac_base>();
    ignore::addRule<surface_border_base, surface_border_base>();
    // add further ignore rules
    ignore::addRule<membrane_part_base, fac_base>();
}

ofApp::~ofApp() {
    delete filamentF;
    filamentF = nullptr;
    delete ccell;
    ccell = nullptr;
    delete surface;
    surface = nullptr;
}

void ofApp::setup() {
    // openframeworks settings
    ofSetFrameRate(10);
    ofBackground(50, 50, 50);
    // set settings
    globals.settings.deltaT = 1;
    globals.settings.sideLength = 500;
    // create grid
    globals.grid = new grid::container(globals.guiMain, globals.settings.sideLength);
    // initialize random
    globals.rndC = new random_container();
    globals.rndC->set_seed();
    // initialize some global variables
    globals.time = 0;
    globals.frameNo = 0;
    // create surface
    surface = new simple_surface(globals, globals.settings.sideLength);
    // create cell components
    membraneF = new functor_cell_membraneCreation(globals); // membrane creation functor for cell
    filamentF = new functor_cell_filamentCreation(globals); // filament creation functor for cell
    // register membrane force/torque functors
    membraneF->register_functor(new functor::membraneSpring(membraneF->get_guiFunctor()));
    // register filament  force/torque functors
    filamentF->register_functor(new functor::ffFriction(filamentF->get_guiFunctor()));
    filamentF->register_functor(new functor::fViscosity(filamentF->get_guiFunctor()));
    // create actual cell
    ccell = new cell(globals, membraneF, filamentF);
}

//--------------------------------------------------------------
void ofApp::update() {
    // calculate scale factor when window is resized
    scale = std::min(ofGetHeight() / (double) globals.settings.sideLength,
                     ofGetWidth() / (double) globals.settings.sideLength);
    // variableUpdate gui variables
    globals.guiC->update();
    // only execute if simulation is not paused
    if (!halt) {
        // variableUpdate global variables
        globals.frameNo++;
        globals.time = globals.frameNo * globals.settings.deltaT;
        // variableUpdate grid
        globals.grid->update_components();
        // make timestep
        ccell->make_timeStep(globals.settings.deltaT);
    }
}

//--------------------------------------------------------------
void ofApp::draw() {
    /* obtain visual objects and draw them */
    std::vector<visual_base *> visualObjs;
    globals.grid->obtain_visualObjs(visualObjs);
    surface->obtain_visualObjs(visualObjs);
    ccell->obtain_visualObjs(visualObjs);
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
