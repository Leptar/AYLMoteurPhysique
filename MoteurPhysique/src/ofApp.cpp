#include "ofApp.h"

#include "ForceGenerator/ForceFriction.h"
#include "ForceGenerator/ForceGravity.h"
#include "Tests/3DVectorTest.h"

#include <algorithm>
#include <sstream>

namespace {
ofColor backgroundTop(12, 16, 32);
ofColor backgroundBottom(4, 6, 12);
}

//--------------------------------------------------------------
void ofApp::setup(){
        projectileConfigs[ProjectileType::Balle] =
        { .masse = 0.02f,  // 20 g (balle d'airsoft)
          .vitesseInitiale = {500, -866, 0},
          .couleur = ofColor::blue,
                .linear = 0.05f, .quadratic = 0.01f,};

        projectileConfigs[ProjectileType::Boulet] =
        { .masse = 5.0f,   // 5 kg
          .vitesseInitiale = {400, -692, 0}, // lourd, donc plus lent
          .couleur = ofColor::gray,
         .linear = 0.f, .quadratic = 0.f,};

        projectileConfigs[ProjectileType::Laser] =
        { .masse = 0.0001f, // quasi nul
          .vitesseInitiale = {3000, -5186, 0},  // constant, pas affecté visuellement par gravité
          .couleur = ofColor::green,
                .linear = 0.f, .quadratic = 0.f,};

        projectileConfigs[ProjectileType::BouleDeFeu] =
        { .masse = 1.0f,   // 1 kg (masse symbolique)
          .vitesseInitiale = {200, -346, 0}, // lent mais chute plus vite
          .couleur = ofColor::red,
        .linear = 0.02f, .quadratic = 0.005f,};

        forces[ForceType::Gravity] = std::make_unique<ForceGravity>();
        forces[ForceType::Friction] = std::make_unique<ForceFriction>();

        float marginX = 120.f;
        float marginY = 140.f;
        blobBounds = ofRectangle(marginX, marginY, std::max(200.f, ofGetWidth() - 2 * marginX), std::max(220.f, ofGetHeight() - 2 * marginY));
        blob.setup(blobBounds);

        lastTime = ofGetElapsedTimeMillis();
}

//--------------------------------------------------------------
void ofApp::update(){
        uint64_t currentTime = ofGetElapsedTimeMillis();
        float dt = (currentTime - lastTime) / 1000.0f; // en secondes
        lastTime = currentTime;
        lastDeltaTime = dt;

        if (dt <= 0.f) {
                return;
        }

        switch (activeScene) {
        case SceneType::Phase1Projectiles:
                updateProjectiles(dt);
                break;
        case SceneType::Phase2Blob:
                blob.update(dt, useVerletBlob, applyGravityBlob, applyFrictionBlob, applySpringsBlob);
                break;
        }
}

//--------------------------------------------------------------
void ofApp::draw(){
        ofBackgroundGradient(backgroundTop, backgroundBottom, OF_GRADIENT_LINEAR);

        switch (activeScene) {
        case SceneType::Phase1Projectiles:
                drawProjectiles();
                break;
        case SceneType::Phase2Blob:
                blob.draw(showSprings, highlightCollisions);
                break;
        }

        if (showHud) {
                drawHud();
        }
}

//--------------------------------------------------------------
void ofApp::updateProjectiles(float dt)
{
        if (projectiles.empty())
                return;

        for (auto& projectile : projectiles) {
                Particule* particule = projectile.particule.get();
                if (applyGravityPhase1) {
                        projectileRegistry.add(particule, forces[ForceType::Gravity].get());
                }
                if (applyFrictionPhase1) {
                        projectileRegistry.add(particule, forces[ForceType::Friction].get());
                }
        }

        projectileRegistry.updateForces(dt);
        projectileRegistry.clear();

        for (auto& projectile : projectiles) {
                projectile.update(dt);
        }
}

//--------------------------------------------------------------
void ofApp::drawProjectiles() const
{
        for (const auto& projectile : projectiles) {
                projectile.draw();
        }
}

//--------------------------------------------------------------
void ofApp::drawHud() const
{
        std::stringstream stream;
        stream << "Delta Time: " << ofToString(lastDeltaTime, 4) << " s\n\n";

        stream << "Scènes (Tab)\n";
        stream << ((activeScene == SceneType::Phase1Projectiles) ? "[x]" : "[ ]") << " Phase 1 - Projectiles (P)\n";
        stream << ((activeScene == SceneType::Phase2Blob) ? "[x]" : "[ ]") << " Phase 2 - Blob (B)\n\n";

        if (activeScene == SceneType::Phase1Projectiles) {
                stream << "Projectiles actifs : " << projectiles.size() << "\n";
                stream << (applyGravityPhase1 ? "[x]" : "[ ]") << " Gravité (G)\n";
                stream << (applyFrictionPhase1 ? "[x]" : "[ ]") << " Traînée (F)\n";
                stream << "Créer un projectile : [1-4]\n";
                stream << "Réinitialiser : R\n";
        } else {
                stream << "Particules du blob : " << blob.particleCount() << "\n";
                stream << "Collisions actives : " << blob.activeCollisionCount() << "\n";
                stream << "Énergie potentielle : " << ofToString(blob.potentialEnergy(), 2) << " J\n";
                stream << (applyGravityBlob ? "[x]" : "[ ]") << " Gravité (G)\n";
                stream << (applyFrictionBlob ? "[x]" : "[ ]") << " Traînée (F)\n";
                stream << (applySpringsBlob ? "[x]" : "[ ]") << " Ressorts (M)\n";
                stream << (useVerletBlob ? "[x]" : "[ ]") << " Intégration Verlet (V)\n";
                stream << (showSprings ? "[x]" : "[ ]") << " Afficher les ressorts (L)\n";
                stream << (highlightCollisions ? "[x]" : "[ ]") << " Mettre en évidence les collisions (C)\n";
                stream << "Déplacer le blob : Flèches ou ZQSD\n";
                stream << "Réinitialiser le blob : R\n";
        }

        stream << "Afficher le HUD : H\n";

        ofDrawBitmapStringHighlight(stream.str(), 20, 30);
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

        if (key == OF_KEY_TAB) {
                activeScene = (activeScene == SceneType::Phase1Projectiles) ? SceneType::Phase2Blob : SceneType::Phase1Projectiles;
        }

        if (key == 'p' || key == 'P') {
                activeScene = SceneType::Phase1Projectiles;
        }
        if (key == 'b' || key == 'B') {
                activeScene = SceneType::Phase2Blob;
        }

        if (key == 'h' || key == 'H') {
                showHud = !showHud;
        }

        if (key == 'g' || key == 'G') {
                if (activeScene == SceneType::Phase1Projectiles) {
                        applyGravityPhase1 = !applyGravityPhase1;
                } else {
                        applyGravityBlob = !applyGravityBlob;
                }
        }

        if (key == 'f' || key == 'F') {
                if (activeScene == SceneType::Phase1Projectiles) {
                        applyFrictionPhase1 = !applyFrictionPhase1;
                } else {
                        applyFrictionBlob = !applyFrictionBlob;
                }
        }

        if (key == 'm' || key == 'M') {
                if (activeScene == SceneType::Phase2Blob) {
                        applySpringsBlob = !applySpringsBlob;
                }
        }

        if (key == 'v' || key == 'V') {
                if (activeScene == SceneType::Phase2Blob) {
                        useVerletBlob = !useVerletBlob;
                }
        }

        if (key == 'l' || key == 'L') {
                if (activeScene == SceneType::Phase2Blob) {
                        showSprings = !showSprings;
                }
        }

        if (key == 'c' || key == 'C') {
                if (activeScene == SceneType::Phase2Blob) {
                        highlightCollisions = !highlightCollisions;
                }
        }

        if (key == 'r' || key == 'R') {
                if (activeScene == SceneType::Phase1Projectiles) {
                        projectiles.clear();
                } else {
                        blob.reset(blobBounds);
                }
        }

        if (activeScene == SceneType::Phase2Blob) {
                if (key == OF_KEY_LEFT || key == 'q' || key == 'Q') {
                        blob.nudge(Vector3D(-1.f, 0.f, 0.f));
                }
                if (key == OF_KEY_RIGHT || key == 'd' || key == 'D') {
                        blob.nudge(Vector3D(1.f, 0.f, 0.f));
                }
                if (key == OF_KEY_UP || key == 'z' || key == 'Z') {
                        blob.nudge(Vector3D(0.f, -1.f, 0.f));
                }
                if (key == OF_KEY_DOWN || key == 's' || key == 'S') {
                        blob.nudge(Vector3D(0.f, 1.f, 0.f));
                }
        }

        if (activeScene == SceneType::Phase1Projectiles) {
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
        float marginX = 120.f;
        float marginY = 140.f;
        float width = std::max(200.f, w - 2 * marginX);
        float height = std::max(220.f, h - 2 * marginY);
        blobBounds = ofRectangle(marginX, marginY, width, height);
        blob.reset(blobBounds);
}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){

}
