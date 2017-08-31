#include "ofApp.h"

//========================================================================
int main() {
    ofSetupOpenGL(1024, 1024, OF_WINDOW);            // <-------- setup the GL context

    // this kicks off the running of my app
    // can be OF_WINDOW or OF_FULLSCREEN
    // pass in width and height too:
    ofRunApp(new ofApp());
}


/*
class base {
    size_t get_type() {
        return typeid(this).hash_code();
    }
};

class child : public base{

};

int main() {
    base *instA = new child();
    // will return id of base*
    size_t id = instA->get_type();
};
*/