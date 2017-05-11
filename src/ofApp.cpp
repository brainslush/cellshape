#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
	ofBackground(200,200,200);
	// create mayors
	Globals.grid = new grid_base(100,sideLength);
	
	// use system uptime as seed
	Globals.rndC = new random_container();
	Globals.rndC->set_seed();
	unsigned long long seed = Globals.rndC->get_seed();

	// Create Components
	Cell = new cell(Globals,250,250,20);
	Cell->create_fillament();
	Surface = new simple_surface(Globals,sideLength);
	Surface->create_facs(0,30,5);
	Globals.grid->update_components();
}

//--------------------------------------------------------------
void ofApp::update(){

}

//--------------------------------------------------------------
void ofApp::draw(){
	std::vector<visual_base*>visualObjs;
	Globals.grid->obtain_visualObjs(visualObjs);
	Surface->obtain_visualObjs(visualObjs);
	Cell->obtain_visualObjs(visualObjs);

	// calculate scale factor when window is resized
	double scale = std::min(ofGetHeight() / (double)sideLength, ofGetWidth() / (double)sideLength);
	// draw elements
	for (auto& it : visualObjs) {
		ofSetColor(it->get_fillColor());
		switch (it->get_type()) {
			case 1:
				ofDrawLine(
						scale* it->get_positions()[0].x,
						scale* it->get_positions()[0].y,
						scale* it->get_positions()[1].x,
						scale* it->get_positions()[1].y
				);
				break;
			case 2:
				ofDrawEllipse(
						scale* it->get_positions()[0].x,
						scale* it->get_positions()[0].y,
						2* scale* it->get_parameters()[0],
						2* scale* it->get_parameters()[1]
				);
				break;
			case 3:
				ofDrawRectangle(
						scale* it->get_positions()[0].x,
						scale* it->get_positions()[0].y,
						scale* (it->get_positions()[2].x - it->get_positions()[0].x),
						scale* (it->get_positions()[2].y - it->get_positions()[0].y)
				);
				break;
			case 4:
				ofDrawTriangle(
						scale* it->get_positions()[0].x,
						scale* it->get_positions()[0].y,
						scale* it->get_positions()[1].x,
						scale* it->get_positions()[1].y,
						scale* it->get_positions()[2].x,
						scale* it->get_positions()[2].y
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
