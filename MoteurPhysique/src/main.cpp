#include "ofMain.h"
#include "ofApp.h"
#include "Tests/3DVectorTest.h"

//========================================================================
int main( ){

	Vector3DTest tester;
	tester.RunAllTests();
        // Utilise ofGLFWWindowSettings pour gérer des options avancées comme le multi-écran.
        ofGLWindowSettings settings;
        settings.setSize(1024, 768);
        settings.windowMode = OF_WINDOW; // peut aussi valoir OF_FULLSCREEN

	auto window = ofCreateWindow(settings);

	ofRunApp(window, std::make_shared<ofApp>());
	ofRunMainLoop();

}