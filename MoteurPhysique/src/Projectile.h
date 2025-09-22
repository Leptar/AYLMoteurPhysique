#pragma once
#include "3DVector.h"
#include "ofColor.h"
#include "Particule.h"

class ofVec3f;

enum class ProjectileType { Balle, Boulet, Laser, BouleDeFeu };

struct ProjectileConfig {
    float masse;
    Vector3D vitesseInitiale;
    ofColor couleur;
};

class Projectile
{
public:
    Particule* particule;
    ProjectileType type;
    ofColor couleur;
    std::vector<ofVec3f> trajectoire;
    
    Projectile(ProjectileType T, const ProjectileConfig& config, const Vector3D& position);

    void update(float deltaTime);

    void draw() const;
    
};