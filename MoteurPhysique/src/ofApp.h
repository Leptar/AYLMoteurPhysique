#pragma once

#include "Blob.h"
#include "ForceGenerator/ParticuleForceRegistry.h"
#include "ForceGenerator/RigidBodyForceGravity.h"
#include "World/RigidBodyBox.h"
#include "World/World.h"
#include "ofMain.h"
#include "Projectile.h"

#include <map>
#include <memory>
#include <vector>

class ofApp : public ofBaseApp{

        public:
                void setup();
                void update();
                void draw();

                void keyPressed(int key);
                void keyReleased(int key);
                void mouseMoved(int x, int y );
                void mouseDragged(int x, int y, int button);
                void mousePressed(int x, int y, int button);
                void mouseReleased(int x, int y, int button);
                void mouseEntered(int x, int y);
                void mouseExited(int x, int y);
                void windowResized(int w, int h);
                void dragEvent(ofDragInfo dragInfo);
                void gotMessage(ofMessage msg);
        private:
                /// Machine à états simple qui identifie la démonstration active.
                enum class SceneType { Phase1Projectiles, Phase2Blob, Phase3Game, Phase4Octree };

                /// Applique toutes les forces configurées aux projectiles actifs avant de les intégrer.
                void updateProjectiles(float dt);
                /// Dessine les repères pour chaque projectile de la première scène.
                void drawProjectiles() const;
                /// Affiche le HUD de debug décrivant la configuration actuelle.
                void drawHud() const;
                /// Traduit l'intention du clavier en une demande de force pour le blob.
                void applyBlobMovementInput(float dt);
                /// Suit l'appui ou le relâchement des touches de déplacement du blob.
                void setBlobMovementKey(int key, bool isPressed);
                /// Réinitialise tous les indicateurs de déplacement du blob, typiquement lors d'un changement de scène.
                void clearBlobMovementKeys();
                std::vector<Projectile> projectiles;
                std::map<ProjectileType, ProjectileConfig> projectileConfigs;

                ParticuleForceRegistry projectileRegistry;
                std::map<ForceType, std::unique_ptr<ParticuleForceGenerator>> forces;

                Blob blob;
                ofRectangle blobBounds;

                std::vector<RigidBodyBox> rigidBodies;
                std::vector<RigidBodyBox> pendingRigidBodySpawns;
                ofEasyCam rigidBodyCamera;
                bool rigidBodyResetRequested = false;

                /// Initialise la scène du jeu (phase 3) et peuple les corps rigides.
                void setupRigidBodyGame();
                /// Ajoute un corps rigide actif et met à jour les compteurs associés.
                void addRigidBodyBox(RigidBodyBox&& box);
                /// Effectue immédiatement la réinitialisation de la scène du jeu.
                void performRigidBodyGameReset();
                /// Fait progresser la simulation du mini-jeu de la phase 3.
                void updateRigidBodyGame(float dt);
                /// Dessine l'aire de jeu ainsi que les corps rigides actifs.
                void drawRigidBodyGame();
                /// Fait apparaître un nouveau pavé à la position du distributeur contrôlé par le joueur.
                void spawnRigidBodyFromDropper();
                /// Replace la scène du jeu dans son état initial.
                void resetRigidBodyGame();
                /// Met à jour l'état de la zone de but pour le corps donné.
                void handleRigidBodyGoal(RigidBodyBox& box);
                /// Affiche la structure de l'octree et les contacts détectés.
                void drawOctreeDebug();
                /// Suit les entrées clavier propres au distributeur de la phase 3.
                void setRigidBodyMovementKey(int key, bool isPressed);
                /// Réinitialise les indicateurs de déplacement du distributeur de la phase 3.
                void clearRigidBodyMovementKeys();

                Vector3D rigidBodyGravity = Vector3D(0.f, -9.81f, 0.f);
                float rigidBodyFloorY = -220.f;
                float rigidBodyBoundsX = 320.f;
                float rigidBodyBoundsZ = 320.f;
                float rigidBodyBounce = 0.55f;
                float rigidBodyFloorFriction = 0.8f;
                Vector3D goalCenter = Vector3D(0.f, -220.f, -80.f);
                float goalSize = 160.f;

                float dropperX = 0.f;
                float dropperZ = 0.f;
                float dropperSpeed = 220.f;
                float dropperSpawnHeight = 260.f;

                bool moveDropperLeft = false;
                bool moveDropperRight = false;
                bool moveDropperForward = false;
                bool moveDropperBackward = false;

                bool applyGravityRigidBodies = true;
                bool drawRigidBodyWireframe = false;
                bool drawOctreeNodes = true;
                bool drawContactPoints = true;

                int rigidBodyScore = 0;
                int rigidBodyLost = 0;
                int totalRigidBodySpawned = 0;

                std::unique_ptr<RigidBodyForceGravity> rigidBodyGravityForce;
                World physicsWorld;

                SceneType activeScene = SceneType::Phase1Projectiles;

                bool applyGravityPhase1 = true;
                bool applyFrictionPhase1 = true;

                bool applyGravityBlob = true;
                bool applyFrictionBlob = true;
                bool applySpringsBlob = true;
                bool useVerletBlob = true;

                bool showHud = true;
                bool showSprings = true;
                bool highlightCollisions = true;

                bool movingLeft = false;
                bool movingRight = false;
                bool movingUp = false;
                bool movingDown = false;

                float lastDeltaTime = 0.f;

                uint64_t lastTime = 0;
};
