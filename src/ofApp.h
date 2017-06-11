#pragma once

#include <functional>
#include "cCell.h"
#include "cSurface.h"
#include "gui.h"
#include "ofMain.h"

#ifndef SRC_OFAPP_H_
#define SRC_OFAPP_H_

class ofApp : public ofBaseApp {
public:

    ofApp();

    ~ofApp();

    void setup();

    void update();

    void draw();

    void keyPressed(int key);

    void keyReleased(int key);

    void mouseMoved(int x, int y);

    void mouseDragged(int x, int y, int button);

    void mousePressed(int x, int y, int button);

    void mouseReleased(int x, int y, int button);

    void mouseEntered(int x, int y);

    void mouseExited(int x, int y);

    void windowResized(int w, int h);

    void dragEvent(ofDragInfo dragInfo);

    void gotMessage(ofMessage msg);

protected:
    mygui::group *guiGroup;
    sGlobalVars Globals;
    functor_cell_filamentCreation *FilamentF;
    simple_surface *Surface;
    cell *Cell;

    bool halt;
    double scale;
    //unsigned &maxFPS;
};

#endif
