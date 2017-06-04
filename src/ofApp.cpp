#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
    // openframeworks settings
    ofSetFrameRate(10);
    ofBackground(200,200,200);
    // create GUI Container
    Globals.guiBase = new mygui::gui();
    // set settings
    Globals.settings.deltaT = 1;
    // create grid
    Globals.grid = new grid_base(Globals.guiBase,100,sideLength);
    // initialize random
    Globals.rndC = new random_container();
    Globals.rndC->set_seed();
    // initialize some global variables
    Globals.time = 0;
    Globals.frameNo = 0;
    // create cell components
    FilamentF = new functor_cell_filamentCreation(Globals,200,0.01,100,500,10); // filament creation functor for cell
    Cell = new cell(Globals,250,250,20,FilamentF);
    Surface = new simple_surface(Globals,sideLength);
    Surface->create_facs(0,30,5);
    Globals.grid->update_components();
}

//--------------------------------------------------------------
void ofApp::update(){
    /* update globals */
    Globals.frameNo ++;
    Globals.time = Globals.frameNo * Globals.settings.deltaT;
    /* update grid */
    Globals.grid->update_components();
    /* make step*/
    Cell->make_timeStep(Globals.settings.deltaT);
}

//--------------------------------------------------------------
void ofApp::draw(){
    /* obtain visual objects */
    std::vector<visual_base*>visualObjs;
    Globals.grid->obtain_visualObjs(visualObjs);
    Surface->obtain_visualObjs(visualObjs);
    Cell->obtain_visualObjs(visualObjs);

    // calculate scale factor when window is resized
    double scale = std::min(ofGetHeight() / (double)sideLength, ofGetWidth() / (double)sideLength);
    // draw elements
    for (auto& it : visualObjs) {
        it->draw(scale);
    }
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){

}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 

}
