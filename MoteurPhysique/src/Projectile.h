#pragma once
#include "3DVector.h"
#include "ofColor.h"
#include "Particule.h"
#include <vector>

class ofVec3f;

enum class ProjectileType { Balle, Boulet, Laser, BouleDeFeu };

struct ProjectileConfig {
    float masse;
    Vector3D vitesseInitiale;
    ofColor couleur;
    float gravityScale;
    float dragCoefficient;
    float damping;
    float minLaunchMultiplier;
    float maxLaunchMultiplier;
};

class Projectile
{
public:
    Particule* particule;
    ProjectileType type;
    ofColor couleur;
    std::vector<ofVec3f> trajectoire;
    ProjectileConfig config;

    Projectile(ProjectileType T,
               const ProjectileConfig& config,
               const Vector3D& position,
               float widthScale,
               float heightScale,
               float launchPowerMultiplier);

    void update(float deltaTime);

    void draw() const;

    void rescale(float widthRatio, float heightRatio);

private:
    float widthScale;
    float heightScale;
    
};
