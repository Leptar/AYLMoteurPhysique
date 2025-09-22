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
                // Collection of projectiles currently simulated and rendered.
                std::vector<Projectile> projectiles;
                // Immutable parameters describing each projectile archetype.
                std::map<ProjectileType, ProjectileConfig> projectileConfigs;

                // Window metrics captured at launch and refreshed after resize events.
                float baseWindowWidth = 0.0f;
                float baseWindowHeight = 0.0f;
                float currentWindowWidth = 0.0f;
                float currentWindowHeight = 0.0f;

                // Last simulation time-step, used for diagnostics.
                float lastDeltaTime = 0.0f;

                // Timestamp of the previous update, in milliseconds.
                uint64_t lastTime = 0;

                // Mouse position expressed in normalized window coordinates.
                float cursorNormalizedX = 0.5f;
                float cursorNormalizedY = 0.5f;
                // Scaling factors derived from the cursor to modulate launch vectors.
                float aimHorizontalScale = 1.0f;
                float aimVerticalScale = 1.0f;

                // Recompute cursor normalization and dependent scaling factors.
                void updateAimFromCursor(int x, int y);
                // Derive the horizontal and vertical multipliers applied to new projectiles.
                void recalculateAimScales();
};
