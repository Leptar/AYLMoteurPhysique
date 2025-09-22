#include "ofApp.h"
#include "Tests/3DVectorTest.h"

#include <cmath>

//--------------------------------------------------------------
void ofApp::setup(){
        const auto makeVelocity = [](float speed, float angleDeg) {
                const float angleRad = ofDegToRad(angleDeg);
                return Vector3D(
                        speed * std::cos(angleRad),
                        -speed * std::sin(angleRad),
                        0.0f);
        };

        projectileConfigs[ProjectileType::Balle] =
        { .masse = 0.02f,  // 20 g (balle d'airsoft)
          .vitesseInitiale = makeVelocity(620.0f, 32.0f),
          .couleur = ofColor::blue };

        projectileConfigs[ProjectileType::Boulet] =
        { .masse = 5.0f,   // 5 kg
          .vitesseInitiale = makeVelocity(340.0f, 55.0f), // lourd, donc plus lent et tir plus courbe
          .couleur = ofColor::gray };

        projectileConfigs[ProjectileType::Laser] =
        { .masse = 0.0001f, // quasi nul
          .vitesseInitiale = makeVelocity(2800.0f, 8.0f),  // tir quasi horizontal
          .couleur = ofColor::green };

        projectileConfigs[ProjectileType::BouleDeFeu] =
        { .masse = 1.0f,   // 1 kg (masse symbolique)
          .vitesseInitiale = makeVelocity(460.0f, 70.0f), // trajectoire très arquée
          .couleur = ofColor::red };

	lastTime = ofGetElapsedTimeMillis();
}

//--------------------------------------------------------------
void ofApp::update(){
	uint64_t currentTime = ofGetElapsedTimeMillis();
	float dt = (currentTime - lastTime) / 1000.0f; // en secondes
	lastTime = currentTime;

	for (auto& p : projectiles) {
		p.update(dt);
	}
	ofDrawBitmapStringHighlight("Delta Time: " + ofToString(dt, 4), 20, 20);
}

//--------------------------------------------------------------
void ofApp::draw(){
	for (auto& p : projectiles) {
		p.draw();
	}
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){

	if (key == OF_KEY_F7) {
		// ouvrir une console si elle n'existe pas déjà
		#ifdef _WIN32
		if (!GetConsoleWindow()) {
			AllocConsole();
			freopen("CONOUT$", "w", stdout);
			freopen("CONOUT$", "w", stderr);
			freopen("CONIN$",  "r", stdin);
		}
		#endif

		Vector3DTest tester;
		char choix;
		do {
			std::cout << "\n=== MENU TESTS Vector3D ===\n"
					  << "1) Addition\n"
					  << "2) Soustraction\n"
					  << "3) Scalaire (*)\n"
					  << "4) Dot\n"
					  << "5) Cross\n"
					  << "6) Normes\n"
					  << "7) Normalize\n"
					  << "A) Tous les tests\n"
					  << "Q) Quitter\n> ";
			std::cin >> choix;
			choix = std::toupper(choix);

			switch (choix) {
			case '1': tester.TestAddition(); break;
			case '2': tester.TestSubtraction(); break;
			case '3': tester.TestScalarProduct(); break;
			case '4': tester.TestDot(); break;
			case '5': tester.TestCross(); break;
			case '6': tester.TestNorms(); break;
			case '7': tester.TestNormalize(); break;
			case 'A': tester.RunAllTests(); break;
			case 'Q': std::cout << "Sortie du menu.\n"; break;
			default:  std::cout << "Choix invalide.\n"; break;
			}
		} while (choix != 'Q');
	}

	if (key == '1') {
		projectiles.emplace_back(
			ProjectileType::Balle, projectileConfigs[ProjectileType::Balle],
			Vector3D(0, ofGetHeight(), 0)
			);
	}
	if (key == '2') {
		projectiles.emplace_back(
			ProjectileType::Boulet, projectileConfigs[ProjectileType::Boulet],
			Vector3D(0, ofGetHeight(), 0)
			);
	}
	if (key == '3') {
		projectiles.emplace_back(
			ProjectileType::Laser, projectileConfigs[ProjectileType::Laser],
			Vector3D(0, ofGetHeight(), 0)
			);
	}
	if (key == '4') {
		projectiles.emplace_back(
			ProjectileType::BouleDeFeu, projectileConfigs[ProjectileType::BouleDeFeu],
			Vector3D(0, ofGetHeight(), 0)
			);
	}
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