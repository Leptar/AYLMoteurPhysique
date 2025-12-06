#include "ofApp.h"

#include "Matrix3Test.h"
#include "Matrix4Test.cpp"
#include "QuaternionTest.h"
#include "ForceGenerator/ForceFriction.h"
#include "ForceGenerator/ForceGravity.h"
#include "Tests/3DVectorTest.h"
#include "glm/vec3.hpp"

#include <algorithm>
#include <cmath>
#include <cctype>
#include <limits>
#include <sstream>
#include <utility>

namespace {
ofColor backgroundTop(12, 16, 32);
ofColor backgroundBottom(4, 6, 12);

ofMatrix4x4 buildTransformMatrix(const Quaternion& orientation, const Vector3D& position)
{
        Quaternion normalized = orientation;
        normalized.normalize();

        float clampedW = ofClamp(normalized.w, -1.f, 1.f);
        float angleRad = 2.f * std::acos(clampedW);
        float sinHalf = std::sqrt(std::max(0.f, 1.f - clampedW * clampedW));

        ofVec3f axis(0.f, 1.f, 0.f);
        if (sinHalf > 1e-4f) {
                axis.set(normalized.x / sinHalf, normalized.y / sinHalf, normalized.z / sinHalf);
        }

        ofQuaternion ofQuat;
        ofQuat.makeRotate(ofRadToDeg(angleRad), axis);

        ofMatrix4x4 transform;
        transform.makeRotationMatrix(ofQuat);
        transform.setTranslation(position.x, position.y, position.z);
        return transform;
}

}

//--------------------------------------------------------------
void ofApp::setup(){
		Matrix3Test TestMatrix;
		TestMatrix.RunAllTests();
		RunAllTestsMatrix4();
		QuaternionTest::RunAllTests();

        // Configure les préréglages de projectiles par défaut pour la première scène.
        projectileConfigs[ProjectileType::Balle] =
        { .masse = 0.02f,  // 20 g (balle d'airsoft)
          .vitesseInitiale = {500, -866, 0},
          .couleur = ofColor::blue,
                .linear = 0.02f, .quadratic = 0.00001f,};

        projectileConfigs[ProjectileType::Boulet] =
        { .masse = 5.0f,   // 5 kg
          .vitesseInitiale = {400, -692, 0}, // lourd, donc plus lent
          .couleur = ofColor::gray,
         .linear = 0.05f, .quadratic = 0.005f,};

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

        // Créer les générateurs de forces partagés pour la scène des projectiles.
        forces[ForceType::Gravity] = std::make_unique<ForceGravity>();
        forces[ForceType::Friction] = std::make_unique<ForceFriction>();

        rigidBodyGravityForce = std::make_unique<RigidBodyForceGravity>(rigidBodyGravity);
        rigidBodyGravityForce->setEnabled(applyGravityRigidBodies);
        physicsWorld.getRigidBodyForceRegistry()->add(rigidBodyGravityForce.get());

        float marginX = 120.f;
        float marginY = 140.f;
        blobBounds = ofRectangle(marginX, marginY, std::max(200.f, ofGetWidth() - 2 * marginX), std::max(220.f, ofGetHeight() - 2 * marginY));
        blob.setup(blobBounds);

        setupRigidBodyGame();

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

        if (rigidBodyGravityForce) {
                rigidBodyGravityForce->setGravity(rigidBodyGravity);
                rigidBodyGravityForce->setEnabled(applyGravityRigidBodies);
        }

        switch (activeScene) {
        case SceneType::Phase1Projectiles:
                // Faire avancer la progression temporelle de la démonstration de projectiles.
                updateProjectiles(dt);
                break;
        case SceneType::Phase2Blob:
                // Récupérer les entrées du joueur et faire avancer la simulation du blob.
                applyBlobMovementInput(dt);
                blob.update(dt, useVerletBlob, applyGravityBlob, applyFrictionBlob, applySpringsBlob);
                break;
        case SceneType::Phase3Game:
        case SceneType::OctreeDebug:
                updateRigidBodyGame(dt);
                break;
        }
}

//--------------------------------------------------------------
void ofApp::draw(){
        ofBackgroundGradient(backgroundTop, backgroundBottom, OF_GRADIENT_LINEAR);

        switch (activeScene) {
        case SceneType::Phase1Projectiles:
                // Dessiner les trajectoires balistiques et leurs repères.
                drawProjectiles();
                break;
        case SceneType::Phase2Blob:
                // Afficher le blob et les diagnostics optionnels.
                blob.draw(showSprings, highlightCollisions);
                break;
        case SceneType::Phase3Game:
                drawRigidBodyGame();
                break;
        case SceneType::OctreeDebug:
                drawOctreeDebug();
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
                        // Enregistrer la gravité pour ce pas de temps.
                        projectileRegistry.add(particule, forces[ForceType::Gravity].get());
                }
                if (applyFrictionPhase1) {
                        // Mettre la traînée en attente pour que les projectiles rapides finissent par ralentir.
                        projectileRegistry.add(particule, forces[ForceType::Friction].get());
                }
        }

        // Appliquer toutes les forces en attente avant d'intégrer.
        projectileRegistry.updateForces(dt);
        projectileRegistry.clear();

        for (auto& projectile : projectiles) {
                // Intégrer chaque projectile via sa routine interne.
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
void ofApp::applyBlobMovementInput(float dt)
{
        if (dt <= 0.f)
                return;

        Vector3D direction(
                (movingRight ? 1.f : 0.f) - (movingLeft ? 1.f : 0.f),
                (movingDown ? 1.f : 0.f) - (movingUp ? 1.f : 0.f),
                0.f);

        // Déléguer au blob afin qu'il traduise l'intention en forces.
        blob.applyMovement(direction, dt);
}

//--------------------------------------------------------------
void ofApp::drawHud() const
{
        std::stringstream stream;
        stream << "Delta Time: " << ofToString(lastDeltaTime, 4) << " s\n\n";

        stream << "Scènes (Tab)\n";
        stream << ((activeScene == SceneType::Phase1Projectiles) ? "[x]" : "[ ]") << " Phase 1 - Projectiles (P)\n";
        stream << ((activeScene == SceneType::Phase2Blob) ? "[x]" : "[ ]") << " Phase 2 - Blob (B)\n\n";
        stream << ((activeScene == SceneType::Phase3Game) ? "[x]" : "[ ]") << " Phase 3 - Jeu de caisses (J)\n";
        stream << ((activeScene == SceneType::OctreeDebug) ? "[x]" : "[ ]") << " Phase 4 - Octree (O)\n\n";

        if (activeScene == SceneType::Phase1Projectiles) {
                stream << "Projectiles actifs : " << projectiles.size() << "\n";
                stream << (applyGravityPhase1 ? "[x]" : "[ ]") << " Gravité (G)\n";
                stream << (applyFrictionPhase1 ? "[x]" : "[ ]") << " Traînée (F)\n";
                stream << "Créer un projectile : [1-4]\n";
                stream << "Réinitialiser : R\n";
        } else if (activeScene == SceneType::Phase2Blob) {
                stream << "Particules du blob : " << blob.particleCount() << "\n";
                stream << "Collisions actives : " << blob.activeCollisionCount() << "\n";
                stream << (applyGravityBlob ? "[x]" : "[ ]") << " Gravité (G)\n";
                stream << (applyFrictionBlob ? "[x]" : "[ ]") << " Traînée (F)\n";
                stream << (applySpringsBlob ? "[x]" : "[ ]") << " Ressorts (M)\n";
                stream << (useVerletBlob ? "[x]" : "[ ]") << " Intégration Verlet (V)\n";
                stream << (showSprings ? "[x]" : "[ ]") << " Afficher les ressorts (L)\n";
                stream << (highlightCollisions ? "[x]" : "[ ]") << " Mettre en évidence les collisions (C)\n";
                stream << "Déplacer le blob : Flèches ou ZQSD\n";
                stream << "Détacher une particule : Backspace\n";
                stream << "Réattacher toutes les particules : Entrée\n";
                stream << "Réinitialiser le blob : R\n";
        } else if (activeScene == SceneType::Phase3Game) {
                int activeCrates = std::max(0, totalRigidBodySpawned - rigidBodyScore - rigidBodyLost);
                stream << "Caisses actives : " << activeCrates << " / " << totalRigidBodySpawned << "\n";
                stream << "Points : " << rigidBodyScore << "\n";
                stream << "Perdues : " << rigidBodyLost << "\n";
                stream << (applyGravityRigidBodies ? "[x]" : "[ ]") << " Gravité (G)\n";
                stream << (drawRigidBodyWireframe ? "[x]" : "[ ]") << " Mode filaire (X)\n";
                stream << "Déplacer le distributeur : Flèches ou ZQSD\n";
                stream << "Lancer une caisse : Espace\n";
                stream << "Réinitialiser : R\n";
        } else if (activeScene == SceneType::OctreeDebug) {
                stream << "Octree : " << (showOctree ? "visible" : "caché") << " (T)\n";
                stream << "Scène basée sur le jeu de caisses\n";
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
                // Basculer entre les scènes de démonstration.
                if (activeScene == SceneType::Phase1Projectiles) {
                        activeScene = SceneType::Phase2Blob;
                } else if (activeScene == SceneType::Phase2Blob) {
                        activeScene = SceneType::Phase3Game;
                } else if (activeScene == SceneType::Phase3Game) {
                        activeScene = SceneType::OctreeDebug;
                } else {
                        activeScene = SceneType::Phase1Projectiles;
                }

                if (activeScene != SceneType::Phase2Blob) {
                        clearBlobMovementKeys();
                }
                if (activeScene != SceneType::Phase3Game && activeScene != SceneType::OctreeDebug) {
                        clearRigidBodyMovementKeys();
                }
        }

        if (key == 'p' || key == 'P') {
                activeScene = SceneType::Phase1Projectiles;
                clearBlobMovementKeys();
                clearRigidBodyMovementKeys();
        }
        if (key == 'b' || key == 'B') {
                activeScene = SceneType::Phase2Blob;
                clearRigidBodyMovementKeys();
        }
        if (key == 'j' || key == 'J') {
                activeScene = SceneType::Phase3Game;
                clearBlobMovementKeys();
                clearRigidBodyMovementKeys();
        }
        if (key == 'o' || key == 'O') {
                activeScene = SceneType::OctreeDebug;
                clearBlobMovementKeys();
        }

        if (key == 'h' || key == 'H') {
                showHud = !showHud;
        }

        if (key == 'g' || key == 'G') {
                if (activeScene == SceneType::Phase1Projectiles) {
                        applyGravityPhase1 = !applyGravityPhase1;
                } else if (activeScene == SceneType::Phase2Blob) {
                        applyGravityBlob = !applyGravityBlob;
                } else if (activeScene == SceneType::Phase3Game || activeScene == SceneType::OctreeDebug) {
                        applyGravityRigidBodies = !applyGravityRigidBodies;
                }
        }

        if (key == 'f' || key == 'F') {
                if (activeScene == SceneType::Phase1Projectiles) {
                        applyFrictionPhase1 = !applyFrictionPhase1;
                } else if (activeScene == SceneType::Phase2Blob) {
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

        if (key == 't' || key == 'T') {
                if (activeScene == SceneType::OctreeDebug) {
                        showOctree = !showOctree;
                }
        }

        if (key == 'x' || key == 'X') {
                if (activeScene == SceneType::Phase3Game) {
                        drawRigidBodyWireframe = !drawRigidBodyWireframe;
                }
        }

        if (key == ' ') {
                if (activeScene == SceneType::Phase3Game) {
                        spawnRigidBodyFromDropper();
                }
        }

        if (key == 'r' || key == 'R') {
                if (activeScene == SceneType::Phase1Projectiles) {
                        // Supprimer tous les projectiles et attendre que l'utilisateur en crée de nouveaux.
                        projectiles.clear();
                } else if (activeScene == SceneType::Phase2Blob) {
                        // Reconstruire le blob dans sa configuration d'origine.
                        blob.reset(blobBounds);
                } else if (activeScene == SceneType::Phase3Game) {
                        resetRigidBodyGame();
                }
        }

        if (activeScene == SceneType::Phase2Blob) {
                if (key == OF_KEY_BACKSPACE) {
                        // Détacher la prochaine particule disponible lorsque Retour arrière est pressé.
                        blob.detachPeripheralParticle();
                }
                if (key == OF_KEY_RETURN
#ifdef OF_KEY_ENTER
                    || key == OF_KEY_ENTER
#endif
                ) {
                        // Rattacher toutes les particules détachées lorsque Entrée est pressée.
                        blob.reattachAllParticles();
                }
        }

        setBlobMovementKey(key, true);
        setRigidBodyMovementKey(key, true);

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
        setBlobMovementKey(key, false);
        setRigidBodyMovementKey(key, false);
}

//--------------------------------------------------------------
void ofApp::setBlobMovementKey(int key, bool isPressed)
{
        if (activeScene != SceneType::Phase2Blob)
                return;

        switch (key) {
        case OF_KEY_LEFT:
                // Les flèches reproduisent la disposition WASD/ZQSD traditionnelle.
                movingLeft = isPressed;
                break;
        case OF_KEY_RIGHT:
                movingRight = isPressed;
                break;
        case OF_KEY_UP:
                movingUp = isPressed;
                break;
        case OF_KEY_DOWN:
                movingDown = isPressed;
                break;
        }

        if (key >= 0 && key <= std::numeric_limits<unsigned char>::max()) {
                int lowered = std::tolower(static_cast<unsigned char>(key));
                switch (lowered) {
                case 'q':
                        // Gérer le clavier AZERTY (ZQSD) tout en restant compatible QWERTY.
                        movingLeft = isPressed;
                        break;
                case 'd':
                        movingRight = isPressed;
                        break;
                case 'z':
                        movingUp = isPressed;
                        break;
                case 's':
                        movingDown = isPressed;
                        break;
                }
        }
}

//--------------------------------------------------------------
void ofApp::clearBlobMovementKeys()
{
        movingLeft = false;
        movingRight = false;
        movingUp = false;
        movingDown = false;
}

//--------------------------------------------------------------
void ofApp::setupRigidBodyGame()
{
	
        rigidBodyCamera.setNearClip(0.1f);
        rigidBodyCamera.setFarClip(4000.f);
        rigidBodyCamera.setPosition(0.f, 360.f, 620.f);
        rigidBodyCamera.lookAt(glm::vec3(0.f, rigidBodyFloorY + 80.f, 0.f));
        rigidBodyCamera.setAutoDistance(false);

        performRigidBodyGameReset();
}

//--------------------------------------------------------------
//--------------------------------------------------------------
void ofApp::addRigidBodyBox(RigidBodyBox&& box)
{
        rigidBodies.push_back(std::move(box));
        physicsWorld.registerRigidBody(rigidBodies.back());
        ++totalRigidBodySpawned;
}

//--------------------------------------------------------------
void ofApp::performRigidBodyGameReset()
{
        rigidBodies.clear();
        rigidBodies.reserve(48);
        pendingRigidBodySpawns.clear();

        rigidBodyScore = 0;
        rigidBodyLost = 0;
        totalRigidBodySpawned = 0;

        dropperX = 0.f;
        dropperZ = 0.f;
        clearRigidBodyMovementKeys();

        goalCenter.y = rigidBodyFloorY;

        physicsWorld.setWorldBounds(AABB(
                Vector3D(-rigidBodyBoundsX - 40.f, rigidBodyFloorY - 120.f, -rigidBodyBoundsZ - 40.f),
                Vector3D(rigidBodyBoundsX + 40.f, rigidBodyFloorY + 520.f, rigidBodyBoundsZ + 40.f)));
        physicsWorld.clearRigidBodies();

        auto initialBoxes = physicsWorld.createRigidBodyGame(30, dropperSpawnHeight, rigidBodyBoundsX, rigidBodyBoundsZ);
        for (auto& box : initialBoxes) {
                addRigidBodyBox(std::move(box));
        }
}

//--------------------------------------------------------------
void ofApp::resetRigidBodyGame()
{
        rigidBodyResetRequested = true;
}

//--------------------------------------------------------------
void ofApp::spawnRigidBodyFromDropper()
{
        Vector3D halfExtents(
                ofRandom(12.f, 22.f),
                ofRandom(14.f, 26.f),
                ofRandom(12.f, 22.f));
        Vector3D dimensions = halfExtents.scalar(2.f);
        float volume = dimensions.x * dimensions.y * dimensions.z;
        float density = 0.00085f;
        float mass = ofClamp(volume * density, 10.f, 52.f);

        Vector3D position(dropperX, dropperSpawnHeight, dropperZ);
        ofColor color = ofColor::fromHsb(ofRandom(0, 255), 220, 240);

        Vector3D initialVel(
                ofRandom(-35.f, 35.f),
                ofRandom(-5.f, 15.f),
                ofRandom(-35.f, 35.f));
        Vector3D angularVel(
                ofRandom(-2.f, 2.f),
                ofRandom(-2.f, 2.f),
                ofRandom(-2.f, 2.f));

        pendingRigidBodySpawns.push_back(physicsWorld.createRigidBodyBox(position, halfExtents, mass, color, initialVel, angularVel));
}

//--------------------------------------------------------------
void ofApp::updateRigidBodyGame(float dt)
{
        if (rigidBodyResetRequested) {
                performRigidBodyGameReset();
                rigidBodyResetRequested = false;
        }

        if (!pendingRigidBodySpawns.empty()) {
                for (RigidBodyBox& box : pendingRigidBodySpawns) {
                        addRigidBodyBox(std::move(box));
                }
                pendingRigidBodySpawns.clear();
        }

        if (dt <= 0.f) {
                return;
        }

        float moveX = ((moveDropperRight ? 1.f : 0.f) - (moveDropperLeft ? 1.f : 0.f)) * dropperSpeed * dt;
        float moveZ = ((moveDropperBackward ? 1.f : 0.f) - (moveDropperForward ? 1.f : 0.f)) * dropperSpeed * dt;

        dropperX = ofClamp(dropperX + moveX, -rigidBodyBoundsX + 20.f, rigidBodyBoundsX - 20.f);
        dropperZ = ofClamp(dropperZ + moveZ, -rigidBodyBoundsZ + 20.f, rigidBodyBoundsZ - 20.f);

        for (auto& box : rigidBodies) {
                if (box.reachedGoal || box.outOfBounds) {
                        continue;
                }

                physicsWorld.applyRigidBodyForces(box.body, dt);

                box.body.integrer(dt);

                applyRigidBodyBounds(box);
                handleRigidBodyGoal(box);

                Vector3D position = box.body.getPosition();
                if (!box.reachedGoal && position.y < rigidBodyFloorY - 420.f) {
                        box.outOfBounds = true;
                        ++rigidBodyLost;
                }
        }

        physicsWorld.detectAndResolveRigidBodyCollisions();
}

//--------------------------------------------------------------
void ofApp::applyRigidBodyBounds(RigidBodyBox& box)
{
        Vector3D position = box.body.getPosition();
        Vector3D velocity = box.body.getVelocite();
        Vector3D angular = box.body.getVelociteAngulaire();

        float radius = box.boundingRadius;
        bool touchedFloor = false;

        if (position.y - radius < rigidBodyFloorY) {
                position.y = rigidBodyFloorY + radius;
                if (velocity.y < 0.f) {
                        velocity.y = -velocity.y * rigidBodyBounce;
                }
                touchedFloor = true;
        }

        if (position.x - radius < -rigidBodyBoundsX) {
                position.x = -rigidBodyBoundsX + radius;
                if (velocity.x < 0.f) {
                        velocity.x = -velocity.x * rigidBodyBounce;
                }
        } else if (position.x + radius > rigidBodyBoundsX) {
                position.x = rigidBodyBoundsX - radius;
                if (velocity.x > 0.f) {
                        velocity.x = -velocity.x * rigidBodyBounce;
                }
        }

        if (position.z - radius < -rigidBodyBoundsZ) {
                position.z = -rigidBodyBoundsZ + radius;
                if (velocity.z < 0.f) {
                        velocity.z = -velocity.z * rigidBodyBounce;
                }
        } else if (position.z + radius > rigidBodyBoundsZ) {
                position.z = rigidBodyBoundsZ - radius;
                if (velocity.z > 0.f) {
                        velocity.z = -velocity.z * rigidBodyBounce;
                }
        }

        if (touchedFloor) {
                velocity.x *= rigidBodyFloorFriction;
                velocity.z *= rigidBodyFloorFriction;
                angular = angular.scalar(rigidBodyFloorFriction);

                if (std::abs(velocity.x) < 1e-2f) velocity.x = 0.f;
                if (std::abs(velocity.y) < 1e-2f) velocity.y = 0.f;
                if (std::abs(velocity.z) < 1e-2f) velocity.z = 0.f;
        }

        box.body.setPosition(position);
        box.body.setVelocite(velocity);
        box.body.setVelociteAngulaire(angular);
}

//--------------------------------------------------------------
void ofApp::handleRigidBodyGoal(RigidBodyBox& box)
{
        if (box.reachedGoal || box.outOfBounds) {
                return;
        }

        float halfGoal = goalSize * 0.5f;
        Vector3D position = box.body.getPosition();
        if (std::abs(position.x - goalCenter.x) <= halfGoal &&
            std::abs(position.z - goalCenter.z) <= halfGoal &&
            position.y - box.boundingRadius <= rigidBodyFloorY + 2.f) {
                box.reachedGoal = true;
                ++rigidBodyScore;

                position.y = rigidBodyFloorY + box.boundingRadius;
                box.body.setPosition(position);
                box.body.setVelocite(Vector3D(0.f, 0.f, 0.f));
                box.body.setVelociteAngulaire(Vector3D(0.f, 0.f, 0.f));
        }
}

//--------------------------------------------------------------
void ofApp::drawRigidBodyGame()
{
        ofEnableDepthTest();
        rigidBodyCamera.begin();

        ofPushStyle();
        ofSetColor(28, 32, 52);
        ofDrawBox(0.f, rigidBodyFloorY - 6.f, 0.f, rigidBodyBoundsX * 2.f + 80.f, 12.f, rigidBodyBoundsZ * 2.f + 80.f);
        ofPopStyle();

        float wallHeight = 220.f;
        float wallThickness = 10.f;

        ofPushStyle();
        ofSetColor(48, 58, 92, 220);
        ofDrawBox(0.f, rigidBodyFloorY + wallHeight * 0.5f, rigidBodyBoundsZ + wallThickness * 0.5f, rigidBodyBoundsX * 2.f, wallHeight, wallThickness);
        ofDrawBox(0.f, rigidBodyFloorY + wallHeight * 0.5f, -rigidBodyBoundsZ - wallThickness * 0.5f, rigidBodyBoundsX * 2.f, wallHeight, wallThickness);
        ofDrawBox(rigidBodyBoundsX + wallThickness * 0.5f, rigidBodyFloorY + wallHeight * 0.5f, 0.f, wallThickness, wallHeight, rigidBodyBoundsZ * 2.f);
        ofDrawBox(-rigidBodyBoundsX - wallThickness * 0.5f, rigidBodyFloorY + wallHeight * 0.5f, 0.f, wallThickness, wallHeight, rigidBodyBoundsZ * 2.f);
        ofPopStyle();

        ofPushStyle();
        ofSetColor(100, 200, 160, 200);
        ofDrawBox(goalCenter.x, rigidBodyFloorY + 1.5f, goalCenter.z, goalSize, 3.f, goalSize);
        ofPopStyle();

        ofPushStyle();
        ofSetLineWidth(2.f);
        ofSetColor(255, 236, 120, 160);
        ofDrawLine(dropperX, rigidBodyFloorY + 1.f, dropperZ, dropperX, dropperSpawnHeight - 14.f, dropperZ);
        ofPopStyle();

        ofPushMatrix();
        ofTranslate(dropperX, dropperSpawnHeight, dropperZ);
        ofPushStyle();
        ofSetColor(250, 210, 70, 220);
        ofDrawBox(0.f, 0.f, 0.f, 36.f, 12.f, 36.f);
        ofPopStyle();
        ofPopMatrix();

        for (const auto& box : rigidBodies) {
                ofPushMatrix();
                ofMatrix4x4 transform = buildTransformMatrix(box.body.getOrientation(), box.body.getPosition());
                ofMultMatrix(transform);

                ofPushStyle();
                ofColor drawColor = box.color;
                if (box.reachedGoal) {
                        drawColor = box.color.getLerped(ofColor::white, 0.4f);
                } else if (box.outOfBounds) {
                        drawColor = ofColor(80, 80, 80, 140);
                }
                ofSetColor(drawColor);

                if (drawRigidBodyWireframe) {
                        ofNoFill();
                        ofDrawBox(0.f, 0.f, 0.f, box.halfExtents.x * 2.f, box.halfExtents.y * 2.f, box.halfExtents.z * 2.f);
                        ofFill();
                } else {
                        ofDrawBox(0.f, 0.f, 0.f, box.halfExtents.x * 2.f, box.halfExtents.y * 2.f, box.halfExtents.z * 2.f);
                }
                ofPopStyle();
                ofPopMatrix();
        }

        rigidBodyCamera.end();
        ofDisableDepthTest();
}

void ofApp::drawOctreeDebug()
{
        drawRigidBodyGame();

        if (!showOctree) {
                return;
        }

        const auto& nodes = physicsWorld.getOctreeNodes();
        if (nodes.empty()) {
                return;
        }

        ofEnableDepthTest();
        rigidBodyCamera.begin();
        ofPushStyle();
        ofNoFill();
        ofSetColor(90, 200, 255, 120);
        for (const auto& node : nodes) {
                Vector3D size = node.max - node.min;
                Vector3D center = node.getCenter();
                ofDrawBox(center.x, center.y, center.z, size.x, size.y, size.z);
        }
        ofPopStyle();
        rigidBodyCamera.end();
        ofDisableDepthTest();
}

//--------------------------------------------------------------
void ofApp::setRigidBodyMovementKey(int key, bool isPressed)
{
        if (activeScene != SceneType::Phase3Game)
                return;

        switch (key) {
        case OF_KEY_LEFT:
                moveDropperLeft = isPressed;
                break;
        case OF_KEY_RIGHT:
                moveDropperRight = isPressed;
                break;
        case OF_KEY_UP:
                moveDropperForward = isPressed;
                break;
        case OF_KEY_DOWN:
                moveDropperBackward = isPressed;
                break;
        }

        if (key >= 0 && key <= std::numeric_limits<unsigned char>::max()) {
                int lowered = std::tolower(static_cast<unsigned char>(key));
                switch (lowered) {
                case 'q':
                case 'a':
                        moveDropperLeft = isPressed;
                        break;
                case 'd':
                        moveDropperRight = isPressed;
                        break;
                case 'z':
                case 'w':
                        moveDropperForward = isPressed;
                        break;
                case 's':
                        moveDropperBackward = isPressed;
                        break;
                }
        }
}

//--------------------------------------------------------------
void ofApp::clearRigidBodyMovementKeys()
{
        moveDropperLeft = false;
        moveDropperRight = false;
        moveDropperForward = false;
        moveDropperBackward = false;
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
