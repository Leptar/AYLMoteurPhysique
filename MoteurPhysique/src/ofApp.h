#pragma once

#include "ofMain.h"
#include "Projectile.h"

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
                static constexpr float BASE_WIDTH = 1024.0f;
                static constexpr float BASE_HEIGHT = 768.0f;
                const float fixedDeltaTime = 1.0f / 60.0f;

                std::vector<Projectile> projectiles;
                std::map<ProjectileType, ProjectileConfig> projectileConfigs;

                uint64_t lastTime = 0;
                float accumulator = 0.0f;
                float widthScale = 1.0f;
                float heightScale = 1.0f;
                float lastFrameDt = 0.0f;

                void updateWindowScale(int w, int h);
                void integratePhysicsStep(float dt);
                float computeLaunchMultiplier(const ProjectileConfig& config) const;
};
