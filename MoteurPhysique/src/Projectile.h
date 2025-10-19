#pragma once
#include "3DVector.h"
#include "ofColor.h"
#include "Particule.h"

#include <memory>

class ofVec3f;

enum class ProjectileType { Balle, Boulet, Laser, BouleDeFeu };

struct ProjectileConfig {
    float masse;
    Vector3D vitesseInitiale;
    ofColor couleur;
	float linear, quadratic;
};

class Projectile
{
public:
    std::unique_ptr<Particule> particule;
    ProjectileType type;
    ofColor couleur;
    std::vector<ofVec3f> trajectoire;

    Projectile(ProjectileType T, const ProjectileConfig& config, const Vector3D& position);

    Projectile(Projectile&&) noexcept = default;
    Projectile& operator=(Projectile&&) noexcept = default;
    Projectile(const Projectile&) = delete;
    Projectile& operator=(const Projectile&) = delete;

    void update(float deltaTime);

    void draw() const;

};
