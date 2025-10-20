#pragma once

#include "Blob.h"
#include "ForceGenerator/ParticuleForceRegistry.h"
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
                enum class SceneType { Phase1Projectiles, Phase2Blob };

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
