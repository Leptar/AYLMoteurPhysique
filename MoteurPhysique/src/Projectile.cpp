#include "Projectile.h"

#include "of3dGraphics.h"
#include "ofGraphics.h"
#include "ofVec3f.h"

Projectile::Projectile(ProjectileType T,
                       const ProjectileConfig& config,
                       const Vector3D& position,
                       float widthScale,
                       float heightScale)
{
    this->widthScale = widthScale;
    this->heightScale = heightScale;

    Vector3D scaledVelocity(
        config.vitesseInitiale.x * widthScale,
        config.vitesseInitiale.y * heightScale,
        config.vitesseInitiale.z
    );

    particule = new Particule(
        position,
        scaledVelocity,
        Vector3D(0, (config.masse) * 981.f * config.gravityScale * heightScale, 0),
        config.masse
        );
    type = T;
    couleur = config.couleur;
    this->config = config;
    particule->setDamping(config.damping);
    float dt0 = 1.0f / 60.0f; // ou ton dt initial
    particule->setOldPosition(position - scaledVelocity.scalar(dt0));

}

void Projectile::update(float deltaTime)
{
    float invM = particule->getInverseMasse();
    if (invM > 0.0f) {
        float m = 1.0f / invM;
        Vector3D totalForce(0.f, 981.f * config.gravityScale * heightScale * m, 0.f);

        if (config.dragCoefficient > 0.0f) {
            Vector3D velocity = particule->getVx();
            Vector3D drag = velocity.scalar(-config.dragCoefficient);
            totalForce = totalForce + drag;
        }

        particule->setForce(totalForce);
    } else {
        particule->setForce(Vector3D(0.f, 0.f, 0.f));
    }

    particule->integrerVerlet(deltaTime);
    Vector3D pos = particule->getPos();
    trajectoire.emplace_back(pos.x, pos.y, pos.z);
    if (trajectoire.size() > 2000) trajectoire.erase(trajectoire.begin());
}

void Projectile::draw() const
{
    Vector3D pos = particule->getPos();
    ofSetColor(couleur);
    ofDrawSphere(ofVec3f(pos.x, pos.y, pos.z), 5); // rayon arbitraire

    // Dessiner la trajectoire
    ofNoFill();
    ofBeginShape();
    for (auto& p : trajectoire) {
        ofVertex(p.x, p.y, p.z);
    }
    ofEndShape(false);
}

void Projectile::rescale(float widthRatio, float heightRatio)
{
    widthScale *= widthRatio;
    heightScale *= heightRatio;

    Vector3D pos = particule->getPos();
    Vector3D oldPos = particule->getOldPos();
    Vector3D vel = particule->getVx();

    pos.x *= widthRatio;
    pos.y *= heightRatio;
    oldPos.x *= widthRatio;
    oldPos.y *= heightRatio;
    vel.x *= widthRatio;
    vel.y *= heightRatio;

    particule->setPosition(pos);
    particule->setOldPosition(oldPos);
    particule->setVitesse(vel);

    for (auto& p : trajectoire) {
        p.x *= widthRatio;
        p.y *= heightRatio;
    }
}
