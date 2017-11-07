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
    registrar::n::registerBase(typeid(base));
    registrar::n::registerType(typeid(base), typeid(components_base));
    // cell container registration
    registrar::n::registerType(typeid(components_base), typeid(cell_base));
    registrar::n::registerType(typeid(cell_base), typeid(cell));
    // cellcomponents base registration
    registrar::n::registerType(typeid(components_base), typeid(cellcomponents_base));
    // crosslinker registration
    registrar::n::registerType(typeid(cellcomponents_base), typeid(crosslinker_base));
    // fillament registration
    registrar::n::registerType(typeid(cellcomponents_base), typeid(filament_base));
    registrar::n::registerType(typeid(filament_base), typeid(actin));
    // volume registration
    registrar::n::registerType(typeid(cellcomponents_base), typeid(volume_base));
    // matrixcomponents registration
    registrar::n::registerType(typeid(components_base), typeid(matrixcomponents_base));
    // fac registration
    registrar::n::registerType(typeid(matrixcomponents_base), typeid(fac_base));
    registrar::n::registerType(typeid(fac_base), typeid(fac));
    // surface border registration
    registrar::n::registerType(typeid(matrixcomponents_base), typeid(surface_border_base));
    registrar::n::registerType(typeid(surface_border_base), typeid(surface_border));
    // membrane registration
    registrar::n::registerType(typeid(cellcomponents_base), typeid(membrane_part_base));
    registrar::n::registerType(typeid(membrane_part_base), typeid(membrane_part));
    registrar::n::registerType(typeid(cellcomponents_base), typeid(membrane_container));
    // add self ignore rules
    ignore::n::addRule(typeid(membrane_part_base), typeid(membrane_part_base));
    ignore::n::addRule(typeid(fac_base), typeid(fac_base));
    ignore::n::addRule(typeid(surface_border_base), typeid(surface_border_base));
    // add further ignore rules
    ignore::n::addRule(typeid(membrane_part_base), typeid(fac_base));
    ignore::n::listRules();
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
    //membraneF->register_functor(new functor::membraneSpring(membraneF->get_guiFunctor()));
    // register filament  force/torque functors
    //filamentF->register_functor(new functor::fmCollision(filamentF->get_guiFunctor()));
    //filamentF->register_functor(new functor::ffFriction(filamentF->get_guiFunctor()));
    //filamentF->register_functor(new functor::fViscosity(filamentF->get_guiFunctor()));
    filamentF->register_functor(new functor::fConstantForce(filamentF->get_guiFunctor()));
    // create actual cell
    ccell = new cell(globals, membraneF, filamentF);
}

//--------------------------------------------------------------
void ofApp::update() {
    //std::cout << "surface_border :: " << typeid(surface_border).name();
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
