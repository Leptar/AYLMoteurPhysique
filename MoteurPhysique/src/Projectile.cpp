#include "Projectile.h"

#include "of3dGraphics.h"
#include "ofGraphics.h"
#include "ofVec3f.h"

Projectile::Projectile(ProjectileType typeValue, const ProjectileConfig& config, const Vector3D& position)
    : particule(std::make_unique<Particule>(
          position,
          config.vitesseInitiale,
          Vector3D::zero(),
          config.masse,
          config.linear,
          config.quadratic)),
      type(typeValue),
      couleur(config.couleur) {
    const float dt0 = 1.0f / 60.0f;
    particule->_oldPos = position - config.vitesseInitiale * dt0;
}

Projectile::~Projectile() = default;

void Projectile::update(float deltaTime)
{
    if (!particule) {
        return;
    }

    particule->integrerEuler(deltaTime);
    const Vector3D pos = particule->_pos;
    trajectoire.emplace_back(pos.x, pos.y, pos.z);
    if (trajectoire.size() > 2000) {
        trajectoire.erase(trajectoire.begin());
    }
}

void Projectile::draw() const
{
    if (!particule) {
        return;
    }

    const Vector3D pos = particule->_pos;
    ofSetColor(couleur);
    ofDrawSphere(ofVec3f(pos.x, pos.y, pos.z), 5);

    ofNoFill();
    ofBeginShape();
    for (const auto& p : trajectoire) {
        ofVertex(p.x, p.y, p.z);
    }
    ofEndShape(false);
}

