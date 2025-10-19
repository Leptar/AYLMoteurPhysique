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
                /// Simple state machine identifying the active demo.
                enum class SceneType { Phase1Projectiles, Phase2Blob };

                /// Apply all configured forces to the active projectiles and integrate them.
                void updateProjectiles(float dt);
                /// Draw helpers for each projectile in the first scene.
                void drawProjectiles() const;
                /// Render the debug HUD describing the current configuration.
                void drawHud() const;
                /// Translate keyboard intent into a force request for the blob.
                void applyBlobMovementInput(float dt);
                /// Track when blob movement keys are pressed or released.
                void setBlobMovementKey(int key, bool isPressed);
                /// Reset all blob movement flags, typically when leaving the scene.
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
