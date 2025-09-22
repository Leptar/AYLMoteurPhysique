#pragma once

#include <map>
#include <vector>

#include "ofMain.h"
#include "Projectile.h"

/// Application principale openFrameworks pilotant la simulation balistique.
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
                /// Ensemble courant des projectiles simulés puis rendus à l'écran.
                std::vector<Projectile> projectiles;
                /// Paramètres invariants caractérisant chaque archétype de projectile.
                std::map<ProjectileType, ProjectileConfig> projectileConfigs;

                /// Dimensions de référence de la fenêtre, mémorisées pour le recalage des trajectoires.
                float baseWindowWidth = 0.0f;
                float baseWindowHeight = 0.0f;
                /// Dimensions effectives de la fenêtre après redimensionnement ou passage plein écran.
                float currentWindowWidth = 0.0f;
                float currentWindowHeight = 0.0f;

                /// Dernier pas de temps simulé, utile aux diagnostics de performance.
                float lastDeltaTime = 0.0f;

                /// Horodatage de la précédente mise à jour en millisecondes.
                uint64_t lastTime = 0;

                /// Position du curseur exprimée dans l'espace normalisé de la fenêtre.
                float cursorNormalizedX = 0.5f;
                float cursorNormalizedY = 0.5f;
                /// Coefficients issus du curseur pour moduler la vitesse de lancement.
                float aimHorizontalScale = 1.0f;
                float aimVerticalScale = 1.0f;

                /// Normalise la position du curseur et actualise les échelles de visée.
                void updateAimFromCursor(int x, int y);
                /// Calcule les multiplicateurs horizontaux et verticaux appliqués aux nouveaux tirs.
                void recalculateAimScales();
};
