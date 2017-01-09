#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
	ofBackground(200,200,200);

	// create mayors
	Grid = new grid_base(2,2);
	// Create Components
	Cell = new cell(Grid,250,250,1000);
	Surface = new simple_surface(Grid,510);
	Surface->create_facs(0,100,10);
}

//--------------------------------------------------------------
void ofApp::update(){

}

//--------------------------------------------------------------
void ofApp::draw(){
	std::vector<visual_base*>visualObjs;
	Grid->obtain_visualObjs(visualObjs);
	//Surface->obtain_visualObjs(visualObjs);
	//Cell->obtain_visualObjs(visualObjs);

	for (auto& it : visualObjs) {
		ofSetColor(it->get_fillColor());
		switch (it->get_type()) {
			case 1:
				ofDrawLine(
						it->get_positions()[0].x,
						it->get_positions()[0].y,
						it->get_positions()[1].x,
						it->get_positions()[1].y
				);
				break;
			case 2:
				ofDrawEllipse(
						it->get_positions()[0].x,
						it->get_positions()[0].y,
						it->get_parameters()[0],
						it->get_parameters()[1]
				);
				break;
			case 3:
				ofDrawRectangle(
						it->get_positions()[0].x,
						it->get_positions()[0].y,
						it->get_positions()[3].x - it->get_positions()[0].x,
						it->get_positions()[3].y - it->get_positions()[0].y
				);
				break;
			case 4:
				ofDrawTriangle(
						it->get_positions()[0].x,
						it->get_positions()[0].y,
						it->get_positions()[1].x,
						it->get_positions()[1].y,
						it->get_positions()[2].x,
						it->get_positions()[2].y
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
