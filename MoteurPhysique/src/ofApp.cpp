#include "ofApp.h"
#include "Tests/3DVectorTest.h"

//--------------------------------------------------------------
void ofApp::setup(){
        ofSetVerticalSync(true);
        ofSetFrameRate(60);

        updateWindowScale(ofGetWidth(), ofGetHeight());

        projectileConfigs[ProjectileType::Balle] =
        { .masse = 0.02f,  // 20 g (balle d'airsoft)
          .vitesseInitiale = {900.f, -120.f, 0.f}, // trajectoire quasi rectiligne
          .couleur = ofColor::blue,
          .gravityScale = 1.0f,
          .dragCoefficient = 0.45f,
          .damping = 0.985f };

        projectileConfigs[ProjectileType::Boulet] =
        { .masse = 5.0f,   // 5 kg
          .vitesseInitiale = {550.f, -900.f, 0.f}, // cloche prononcée
          .couleur = ofColor::gray,
          .gravityScale = 1.15f,
          .dragCoefficient = 0.05f,
          .damping = 0.995f };

        projectileConfigs[ProjectileType::Laser] =
        { .masse = 0.01f,
          .vitesseInitiale = {2500.f, 0.f, 0.f},  // trajectoire parfaitement droite
          .couleur = ofColor::green,
          .gravityScale = 0.0f,
          .dragCoefficient = 0.0f,
          .damping = 1.0f };

        projectileConfigs[ProjectileType::BouleDeFeu] =
        { .masse = 1.0f,   // 1 kg (masse symbolique)
          .vitesseInitiale = {260.f, -220.f, 0.f}, // lente et portée courte
          .couleur = ofColor::red,
          .gravityScale = 0.8f,
          .dragCoefficient = 0.25f,
          .damping = 0.975f };

        lastTime = ofGetElapsedTimeMillis();
        accumulator = 0.0f;
}

//--------------------------------------------------------------
void ofApp::update(){
        uint64_t currentTime = ofGetElapsedTimeMillis();
        float dt = (currentTime - lastTime) / 1000.0f; // en secondes
        lastTime = currentTime;
        lastFrameDt = dt;

        accumulator += dt;
        float maxAccumulator = fixedDeltaTime * 5.0f;
        if (accumulator > maxAccumulator) {
                accumulator = maxAccumulator;
        }

        while (accumulator >= fixedDeltaTime) {
                integratePhysicsStep(fixedDeltaTime);
                accumulator -= fixedDeltaTime;
        }
}

//--------------------------------------------------------------
void ofApp::draw(){
        for (auto& p : projectiles) {
                p.draw();
        }

        ofDrawBitmapStringHighlight("Delta Time: " + ofToString(lastFrameDt, 4), 20, 20);
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
                        Vector3D(0, ofGetHeight(), 0),
                        widthScale, heightScale
                        );
        }
        if (key == '2') {
                projectiles.emplace_back(
                        ProjectileType::Boulet, projectileConfigs[ProjectileType::Boulet],
                        Vector3D(0, ofGetHeight(), 0),
                        widthScale, heightScale
                        );
        }
        if (key == '3') {
                projectiles.emplace_back(
                        ProjectileType::Laser, projectileConfigs[ProjectileType::Laser],
                        Vector3D(0, ofGetHeight(), 0),
                        widthScale, heightScale
                        );
        }
        if (key == '4') {
                projectiles.emplace_back(
                        ProjectileType::BouleDeFeu, projectileConfigs[ProjectileType::BouleDeFeu],
                        Vector3D(0, ofGetHeight(), 0),
                        widthScale, heightScale
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
        updateWindowScale(w, h);
}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){

}

void ofApp::updateWindowScale(int w, int h){
        float newWidthScale = static_cast<float>(w) / BASE_WIDTH;
        float newHeightScale = static_cast<float>(h) / BASE_HEIGHT;

        float widthRatio = (widthScale > 0.0f) ? (newWidthScale / widthScale) : 1.0f;
        float heightRatio = (heightScale > 0.0f) ? (newHeightScale / heightScale) : 1.0f;

        widthScale = newWidthScale;
        heightScale = newHeightScale;

        for (auto& projectile : projectiles) {
                projectile.rescale(widthRatio, heightRatio);
        }
}

void ofApp::integratePhysicsStep(float dt){
        for (auto& p : projectiles) {
                p.update(dt);
        }
}
