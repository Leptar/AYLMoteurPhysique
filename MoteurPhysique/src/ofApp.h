#pragma once

#include <map>
#include <vector>

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
                std::vector<Projectile> projectiles;
                std::map<ProjectileType, ProjectileConfig> projectileConfigs;

                float baseWindowWidth = 0.0f;
                float baseWindowHeight = 0.0f;
                float currentWindowWidth = 0.0f;
                float currentWindowHeight = 0.0f;

                float lastDeltaTime = 0.0f;

                uint64_t lastTime = 0;
};
