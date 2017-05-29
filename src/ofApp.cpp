#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
    // of Settings
    ofSetFrameRate(10);
    // set settings
    Globals.settings.showGrid = true;
    Globals.settings.showGridOccupation = true;
    Globals.settings.deltaT = 1;

    ofBackground(200,200,200);
    // create mayors
    Globals.grid = new grid_base(Globals.settings,100,sideLength);

    // use system uptime as seed
    Globals.rndC = new random_container();
    Globals.rndC->set_seed();
    unsigned long long seed = Globals.rndC->get_seed();
    Globals.time = 0;
    Globals.frameNo = 0;

    // Create Components
    Cell = new cell(Globals,250,250,20);
    Cell->create_fillament();
    Surface = new simple_surface(Globals,sideLength);
    Surface->create_facs(0,30,5);
    Globals.grid->update_components();
}

//--------------------------------------------------------------
void ofApp::update(){
    Globals.frameNo = ofGetFrameNum();
    Globals.time = Globals.frameNo * Globals.settings.deltaT;
}

//--------------------------------------------------------------
void ofApp::draw(){
    /*update grid*/
    Globals.grid->update_components();

    /*obtain visual objects*/
    std::vector<visual_base*>visualObjs;
    Globals.grid->obtain_visualObjs(visualObjs);
    Surface->obtain_visualObjs(visualObjs);
    Cell->obtain_visualObjs(visualObjs);

    Cell->make_timeStep(Globals.settings.deltaT);

    // calculate scale factor when window is resized
    double scale = std::min(ofGetHeight() / (double)sideLength, ofGetWidth() / (double)sideLength);
    // draw elements
    for (auto& it : visualObjs) {
        ofSetColor(it->get_fillColor());
        switch (it->get_type()) {
            case 1:
                ofDrawLine(
                    scale* it->get_positions()[0](0),
                    scale* it->get_positions()[0](1),
                    scale* it->get_positions()[1](0),
                    scale* it->get_positions()[1](1)
                );
            break;
            case 2:
                ofDrawEllipse(
                    scale* it->get_positions()[0](0),
                    scale* it->get_positions()[0](1),
                    2* scale* it->get_parameters()[0],
                    2* scale* it->get_parameters()[1]
                );
            break;
            case 3:
                ofDrawRectangle(
                    scale* it->get_positions()[0](0),
                    scale* it->get_positions()[0](1),
                    scale* (it->get_positions()[2](0) - it->get_positions()[0](0)),
                    scale* (it->get_positions()[2](1) - it->get_positions()[0](1))
                );
            break;
            case 4:
                ofDrawTriangle(
                    scale* it->get_positions()[0](0),
                    scale* it->get_positions()[0](1),
                    scale* it->get_positions()[1](0),
                    scale* it->get_positions()[1](1),
                    scale* it->get_positions()[2](0),
                    scale* it->get_positions()[2](1)
                );
            break;
            case 5:

            break;
        }
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
