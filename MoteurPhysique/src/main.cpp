#include "ofMain.h"
#include "ofApp.h"
#include "Tests/3DVectorTest.h"

//==========================================================================
int main() {
    Vector3DTest tester;
    tester.RunAllTests();

    // Configuration de la fenêtre principale (peut être basculée en plein écran).
    ofGLWindowSettings settings;
    settings.setSize(1024, 768);
    settings.windowMode = OF_WINDOW;

    auto window = ofCreateWindow(settings);

    ofRunApp(window, std::make_shared<ofApp>());
    ofRunMainLoop();
}

