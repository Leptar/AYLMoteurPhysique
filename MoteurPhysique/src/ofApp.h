#pragma once

#include <map>
#include <memory>
#include <vector>

#include "3DVector.h"
#include "Projectile.h"
#include "World.h"
#include "ofMain.h"

#include "ForceGenerator/ForceFriction.h"
#include "ForceGenerator/ForceGravity.h"
#include "ForceGenerator/ParticuleForceRegistry.h"

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
                enum class Level { Projectiles, Blob };

                void updateProjectiles(float dt);
                void drawProjectiles() const;
                void updateBlob(float dt);
                void drawBlob() const;
                void rebuildBlobObstacles(int width, int height);
                void toggleLevel();

                World world;
                std::vector<Blob::Obstacle> blobObstacles;
                uint64_t lastTime = 0;
                bool moveUp = false;
                bool moveDown = false;
                bool moveLeft = false;
                bool moveRight = false;

                float hudDisplacement = 0.0f;
                float hudVelocity = 0.0f;
                std::size_t hudDisplayedCount = 0;
                float lastDeltaTime = 0.0f;

                Level currentLevel = Level::Projectiles;
                std::map<ProjectileType, ProjectileConfig> projectileConfigs;
                std::vector<Projectile> projectiles;
                std::unique_ptr<ParticuleForceRegistry> registreClass;
                std::map<ForceType, std::unique_ptr<ParticuleForceGenerator>> forces;
};
