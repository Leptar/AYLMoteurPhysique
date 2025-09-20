#include "Projectile.h"

#include "of3dGraphics.h"
#include "ofGraphics.h"
#include "ofVec3f.h"

Projectile::Projectile(ProjectileType T, const ProjectileConfig& config, const Vector3D& position)
{
    particule = new Particule(
        position,
        config.vitesseInitiale,
        Vector3D(0, (config.masse)*981.f, 0),
        config.masse
        );
    type = T;
    couleur = config.couleur;
    float dt0 = 1.0f / 60.0f; // ou ton dt initial
    particule->setOldPosition(position - config.vitesseInitiale.scalar(dt0));
    
}

void Projectile::update(float deltaTime)
{
    float invM = particule->getInverseMasse();
    if (invM > 0.0f) {
        float m = 1.0f / invM;
        particule->setForce(Vector3D(0.f, 981.f * m, 0.f));
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
