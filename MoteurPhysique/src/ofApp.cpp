#include "ofApp.h"

#include "ForceFriction.h"
#include "ForceGravity.h"
#include "Tests/3DVectorTest.h"

//--------------------------------------------------------------
void ofApp::setup(){
	projectileConfigs[ProjectileType::Balle] =
	{ .masse = 0.02f,  // 20 g (balle d'airsoft)
	  .vitesseInitiale = {500, -866, 0},
	  .couleur = ofColor::blue };

	projectileConfigs[ProjectileType::Boulet] =
	{ .masse = 5.0f,   // 5 kg
	  .vitesseInitiale = {400, -692, 0}, // lourd, donc plus lent
	  .couleur = ofColor::gray };

	projectileConfigs[ProjectileType::Laser] =
	{ .masse = 0.0001f, // quasi nul
	  .vitesseInitiale = {3000, -5186, 0},  // constant, pas affecté visuellement par gravité
	  .couleur = ofColor::green };

	projectileConfigs[ProjectileType::BouleDeFeu] =
	{ .masse = 1.0f,   // 1 kg (masse symbolique)
	  .vitesseInitiale = {200, -346, 0}, // lent mais chute plus vite
	  .couleur = ofColor::red };

	registreClass = new ParticuleForceRegistry();
	Forces[ForceType::Gravity] = new ForceGravity();
	Forces[ForceType::Friction] = new ForceFriction();

	lastTime = ofGetElapsedTimeMillis();
}

//--------------------------------------------------------------
void ofApp::update(){
	uint64_t currentTime = ofGetElapsedTimeMillis();
	float dt = (currentTime - lastTime) / 1000.0f; // en secondes
	lastTime = currentTime;

	for (auto& p : projectiles) {
		registreClass->add(p.particule, Forces[ForceType::Gravity]);
		registreClass->add(p.particule, Forces[ForceType::Friction]);
	}
	registreClass->updateForces(dt);
	registreClass->clear();

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
