#include "ofApp.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <iostream>

#include "Tests/3DVectorTest.h"

namespace {
constexpr float kControlStrength = 420.0f;
constexpr float kHudOmega = 8.0f;
constexpr float kHudDamping = 0.6f;
}

//--------------------------------------------------------------
void ofApp::setup(){
        ofSetFrameRate(60);
        ofBackground(18, 18, 24);

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

        registreClass = std::make_unique<ParticuleForceRegistry>();
        forces[ForceType::Gravity] = std::make_unique<ForceGravity>();
        forces[ForceType::Friction] = std::make_unique<ForceFriction>(0.0f, 0.0f);

        world.setup(ofRectangle(0, 0, ofGetWidth(), ofGetHeight()));
        rebuildBlobObstacles(ofGetWidth(), ofGetHeight());

        lastTime = ofGetElapsedTimeMillis();
        hudDisplayedCount = world.getAttachedCount();
}

//--------------------------------------------------------------
void ofApp::update(){
        uint64_t currentTime = ofGetElapsedTimeMillis();
        float dt = (currentTime - lastTime) / 1000.0f;
        if (dt <= 0.0f) {
                dt = 1.0f / 60.0f;
        }
        lastTime = currentTime;
        lastDeltaTime = dt;

        if (currentLevel == Level::Projectiles) {
                updateProjectiles(dt);
        } else {
                updateBlob(dt);
        }
}

//--------------------------------------------------------------
void ofApp::draw(){
        if (currentLevel == Level::Projectiles) {
                drawProjectiles();
        } else {
                drawBlob();
        }
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
        if (key == OF_KEY_TAB) {
                toggleLevel();
                return;
        }

        if (currentLevel == Level::Blob) {
                if (key == 'z' || key == 'Z') moveUp = true;
                if (key == 's' || key == 'S') moveDown = true;
                if (key == 'q' || key == 'Q') moveLeft = true;
                if (key == 'd' || key == 'D') moveRight = true;

                if (key == ' ') {
                        moveUp = true;
                }

                if (key == 'a' || key == 'A') {
                        world.splitBlob();
                }
                if (key == 'e' || key == 'E') {
                        world.mergeBlob();
                }
                return;
        }

        if (key == OF_KEY_F7) {
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
        if (currentLevel != Level::Blob) {
                return;
        }

        if (key == 'z' || key == 'Z') moveUp = false;
        if (key == 's' || key == 'S') moveDown = false;
        if (key == 'q' || key == 'Q') moveLeft = false;
        if (key == 'd' || key == 'D') moveRight = false;
        if (key == ' ') moveUp = false;
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

        world.setup(ofRectangle(0, 0, w, h));
        rebuildBlobObstacles(w, h);
        hudDisplayedCount = world.getAttachedCount();
        hudDisplacement = 0.0f;
        hudVelocity = 0.0f;
}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){
}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){
}

void ofApp::updateProjectiles(float dt) {
        if (!registreClass) {
                return;
        }

        for (auto& projectile : projectiles) {
                for (auto& forceEntry : forces) {
                        registreClass->add(projectile.getParticule(), forceEntry.second.get());
                }
        }

        registreClass->updateForces(dt);
        registreClass->clear();

        for (auto& projectile : projectiles) {
                projectile.update(dt);
        }
}

void ofApp::drawProjectiles() const {
        for (const auto& projectile : projectiles) {
                projectile.draw();
        }

        ofPushStyle();
        ofSetColor(255);
        ofDrawBitmapStringHighlight("Niveau : Projectiles", 20, 30);
        ofDrawBitmapStringHighlight("1-4 : tirer des projectiles", 20, 50);
        ofDrawBitmapStringHighlight("F7 : tests Vector3D", 20, 70);
        ofDrawBitmapStringHighlight("TAB : changer de niveau", 20, 90);
        ofDrawBitmapStringHighlight("Delta Time: " + ofToString(lastDeltaTime, 4), 20, 110);
        ofPopStyle();
}

void ofApp::updateBlob(float dt) {
        Vector3D control = Vector3D::zero();
        if (moveUp) control.y -= kControlStrength;
        if (moveDown) control.y += kControlStrength;
        if (moveLeft) control.x -= kControlStrength;
        if (moveRight) control.x += kControlStrength;
        world.setControlAcceleration(control);

        world.update(dt);

        std::size_t attached = world.getAttachedCount();
        if (attached != hudDisplayedCount) {
                hudDisplayedCount = attached;
                hudDisplacement += 1.0f;
                hudVelocity = -2.5f;
        }

        float acceleration = -kHudOmega * kHudOmega * hudDisplacement - 2.0f * kHudDamping * kHudOmega * hudVelocity;
        hudVelocity += acceleration * dt;
        hudDisplacement += hudVelocity * dt;

        if (std::abs(hudDisplacement) < 1e-4f && std::abs(hudVelocity) < 1e-4f) {
                hudDisplacement = 0.0f;
                hudVelocity = 0.0f;
        }
}

void ofApp::drawBlob() const {
        world.draw();

        ofPushStyle();
        ofSetColor(255);
        ofDrawBitmapStringHighlight("Niveau : Blob", 20, 30);
        ofDrawBitmapStringHighlight("Z/Q/S/D : bouger le blob", 20, 50);
        ofDrawBitmapStringHighlight("Espace : saut", 20, 70);
        ofDrawBitmapStringHighlight("A : séparer", 20, 90);
        ofDrawBitmapStringHighlight("E : fusionner", 20, 110);
        ofDrawBitmapStringHighlight("TAB : changer de niveau", 20, 130);

        float scale = 1.0f + ofClamp(hudDisplacement * 0.18f, -0.3f, 0.3f);
        ofPushMatrix();
        ofTranslate(20, 160);
        ofScale(scale, scale);
        ofDrawBitmapStringHighlight("Particules connectées : " + ofToString(hudDisplayedCount), 0, 0);
        ofPopMatrix();
        ofPopStyle();
}

void ofApp::rebuildBlobObstacles(int width, int height) {
        blobObstacles.clear();
        if (width <= 0 || height <= 0) {
                world.setObstacles(blobObstacles);
                return;
        }

        float minAxis = std::min(width, height);
        float tallWidth = minAxis * 0.08f;
        float tallHeight = minAxis * 0.26f;
        float wideWidth = minAxis * 0.22f;
        float wideHeight = minAxis * 0.07f;
        float circleRadius = minAxis * 0.07f;
        float smallCircle = circleRadius * 0.65f;

        blobObstacles.push_back(Blob::Obstacle::creerRectangle(
                ofRectangle(width * 0.22f - tallWidth * 0.5f, height * 0.62f - tallHeight * 0.5f, tallWidth, tallHeight)));
        blobObstacles.push_back(Blob::Obstacle::creerRectangle(
                ofRectangle(width * 0.58f - wideWidth * 0.5f, height * 0.42f - wideHeight * 0.5f, wideWidth, wideHeight)));
        blobObstacles.push_back(Blob::Obstacle::creerCercle(
                Vector3D(width * 0.35f, height * 0.32f, 0.0f), circleRadius));
        blobObstacles.push_back(Blob::Obstacle::creerCercle(
                Vector3D(width * 0.75f, height * 0.68f, 0.0f), smallCircle));

        world.setObstacles(blobObstacles);
}

void ofApp::toggleLevel() {
        if (currentLevel == Level::Projectiles) {
                currentLevel = Level::Blob;
                moveUp = moveDown = moveLeft = moveRight = false;
                hudDisplayedCount = world.getAttachedCount();
                hudDisplacement = 0.0f;
                hudVelocity = 0.0f;
        } else {
                currentLevel = Level::Projectiles;
                moveUp = moveDown = moveLeft = moveRight = false;
        }
}
