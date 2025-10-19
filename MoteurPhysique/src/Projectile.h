#pragma once

#include "3DVector.h"

#include <memory>
#include <vector>

#include "Particule.h"
#include "ofColor.h"

class ofVec3f;

enum class ProjectileType { Balle, Boulet, Laser, BouleDeFeu };

struct ProjectileConfig {
    float masse;
    Vector3D vitesseInitiale;
    ofColor couleur;
    float linear;
    float quadratic;
};

// ---------------------------------------------------------------------------
// Représente un projectile unique soumis à la gravité et affiché sous forme de
// sphère. Le suivi de trajectoire permet de visualiser la courbe.
// ---------------------------------------------------------------------------
class Projectile {
public:
    Projectile(ProjectileType type, const ProjectileConfig& config, const Vector3D& position);
    ~Projectile();

    void update(float deltaTime);
    void draw() const;

    Particule* getParticule() const { return particule.get(); }

private:
    std::unique_ptr<Particule> particule;
    ProjectileType type;
    ofColor couleur;
    std::vector<ofVec3f> trajectoire;
};

