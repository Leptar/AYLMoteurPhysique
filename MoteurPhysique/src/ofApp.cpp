#include "ofApp.h"
#include "Tests/3DVectorTest.h"

#include <cmath>

//--------------------------------------------------------------
void ofApp::setup(){
        const auto makeVelocityFromEnergy = [](float mass, float energyJoules, float angleDeg) {
                const float speed = (mass > 0.0f) ? std::sqrt(2.0f * energyJoules / mass) : 0.0f;
                const float angleRad = ofDegToRad(angleDeg);
                return Vector3D(
                        speed * std::cos(angleRad),
                        -speed * std::sin(angleRad),
                        0.0f);
        };

        const float masseBalle = 0.02f;
        const float masseBoulet = 5.0f;
        const float masseLaser = 0.0001f;
        const float masseBouleFeu = 1.0f;

        projectileConfigs[ProjectileType::Balle] =
        { .masse = masseBalle,  // 20 g (balle d'airsoft)
          .vitesseInitiale = makeVelocityFromEnergy(masseBalle, 3844.0f, 32.0f),
          .couleur = ofColor::blue };

        projectileConfigs[ProjectileType::Boulet] =
        { .masse = masseBoulet,   // 5 kg
          .vitesseInitiale = makeVelocityFromEnergy(masseBoulet, 289000.0f, 55.0f), // lourd, donc plus lent et tir plus courbe
          .couleur = ofColor::gray };

        projectileConfigs[ProjectileType::Laser] =
        { .masse = masseLaser, // quasi nul
          .vitesseInitiale = makeVelocityFromEnergy(masseLaser, 392.0f, 8.0f),  // tir quasi horizontal
          .couleur = ofColor::green };

        projectileConfigs[ProjectileType::BouleDeFeu] =
        { .masse = masseBouleFeu,   // 1 kg (masse symbolique)
          .vitesseInitiale = makeVelocityFromEnergy(masseBouleFeu, 105800.0f, 70.0f), // trajectoire très arquée
          .couleur = ofColor::red };

        baseWindowWidth = ofGetWidth();
        baseWindowHeight = ofGetHeight();
        currentWindowWidth = baseWindowWidth;
        currentWindowHeight = baseWindowHeight;
        lastTime = ofGetElapsedTimeMillis();
}

//--------------------------------------------------------------
void ofApp::update(){
        uint64_t currentTime = ofGetElapsedTimeMillis();
        float dt = (currentTime - lastTime) / 1000.0f; // en secondes
        lastTime = currentTime;
        lastDeltaTime = dt;

        for (auto& p : projectiles) {
                p.update(dt);
        }
}

//--------------------------------------------------------------
void ofApp::draw(){
        const float scaleX = (baseWindowWidth > 0.0f) ? currentWindowWidth / baseWindowWidth : 1.0f;
        const float scaleY = (baseWindowHeight > 0.0f) ? currentWindowHeight / baseWindowHeight : 1.0f;

        ofPushMatrix();
        ofScale(scaleX, scaleY);
        for (auto& p : projectiles) {
                p.draw();
        }
        ofPopMatrix();

        ofDrawBitmapStringHighlight("Delta Time: " + ofToString(lastDeltaTime, 4), 20, 20);
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){

        if (key == 'f' || key == 'F') {
                ofToggleFullscreen();
                return;
        }

        const auto spawnProjectile = [&](ProjectileType type) {
                const auto configIt = projectileConfigs.find(type);
                if (configIt == projectileConfigs.end()) {
                        return;
                }

                const float spawnY = (baseWindowHeight > 0.0f) ? baseWindowHeight : static_cast<float>(ofGetHeight());

                projectiles.emplace_back(
                        type,
                        configIt->second,
                        Vector3D(0, spawnY, 0));
        };

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
                spawnProjectile(ProjectileType::Balle);
        }
        if (key == '2') {
                spawnProjectile(ProjectileType::Boulet);
        }
        if (key == '3') {
                spawnProjectile(ProjectileType::Laser);
        }
        if (key == '4') {
                spawnProjectile(ProjectileType::BouleDeFeu);
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
        if (w <= 0 || h <= 0) {
                return;
        }

        currentWindowWidth = static_cast<float>(w);
        currentWindowHeight = static_cast<float>(h);
}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){

}