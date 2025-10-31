#include "Blob.h"

#include "3DVector.h"
#include "ofGraphics.h"
#include "ofUtils.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace {
constexpr float kCoreSpringStiffness = 6.0f;
constexpr float kOuterSpringStiffness = 3.0f;
constexpr float kParticleRadius = 16.0f;
constexpr float kMovementImpulse = 22.0f;
constexpr float kMovementBrake = 14.0f;
constexpr std::size_t kPeripheralParticleCount = 8;
constexpr float kTwoPi = 6.28318530717958647692f;
}

Blob::Blob()
{
    // Les générateurs de gravité et de frottement sont partagés par toutes les particules du blob.
    gravity = std::make_unique<ForceGravity>();
    friction = std::make_unique<ForceFriction>();
}

void Blob::setup(const ofRectangle& bounds)
{
    // Création initiale de la disposition du blob à l'intérieur des limites demandées.
    buildBlob(bounds);
}

void Blob::reset(const ofRectangle& bounds)
{
    // Vider tous les caches d'exécution avant de reconstruire le blob depuis zéro.
    centerParticle = nullptr;
    particles.clear();
    springGenerators.clear();
    springBindings.clear();
    springConnections.clear();
    collidingParticles.clear();
    detachedParticles.clear();
    collisionSystem.clear();
    registry.clear();
    buildBlob(bounds);
}

void Blob::setBounds(const ofRectangle& bounds)
{
    // Permet de mettre à jour la zone de jeu (par exemple lors du redimensionnement de la fenêtre).
    playArea = bounds;
}

void Blob::buildBlob(const ofRectangle& bounds)
{
    playArea = bounds;
    detachedParticles.clear();

    Vector3D center(bounds.getCenter().x, bounds.getCenter().y, 0.f);
    float reach = std::min(bounds.getWidth(), bounds.getHeight()) * 0.12f;
    float particleRadius = std::min(kParticleRadius, reach * 0.4f);

    auto createParticle = [&](const Vector3D& position) {
        // Utilitaire pour centraliser la création par défaut des particules.
        float mass = 1.0f;
        float linear = 0.15f;
        float quadratic = 0.01f;
        particles.emplace_back(std::make_unique<Particule>(position, Vector3D(0, 0, 0), Vector3D(0, 0, 0), mass, linear, quadratic, particleRadius));
        return particles.back().get();
    };

    centerParticle = nullptr;
    centerParticle = createParticle(center);

    std::vector<Particule*> peripheralParticles;
    peripheralParticles.reserve(kPeripheralParticleCount);

    if (kPeripheralParticleCount > 0) {
        float ringRadius = std::max(reach, particleRadius * 3.f);
        for (std::size_t i = 0; i < kPeripheralParticleCount; ++i) {
            float angle = (static_cast<float>(i) / static_cast<float>(kPeripheralParticleCount)) * kTwoPi;
            Vector3D offset(std::cos(angle) * ringRadius, std::sin(angle) * ringRadius, 0.f);
            peripheralParticles.push_back(createParticle(center + offset));
        }
    }

    auto connectSpring = [&](Particule* a, Particule* b, float stiffness) {
        if (!a || !b)
            return;

        float restLength = (a->_pos - b->_pos).GetNorm();
        springConnections.push_back({a, b, restLength});

        // Chaque ressort logique est représenté par deux générateurs, un par direction.
        auto springAB = std::make_unique<ForceRessortParticule>(b, stiffness, restLength);
        springBindings.push_back({a, b, springAB.get()});
        springGenerators.push_back(std::move(springAB));

        auto springBA = std::make_unique<ForceRessortParticule>(a, stiffness, restLength);
        springBindings.push_back({b, a, springBA.get()});
        springGenerators.push_back(std::move(springBA));
    };

    for (Particule* peripheral : peripheralParticles) {
        connectSpring(centerParticle, peripheral, kCoreSpringStiffness);
    }

    if (peripheralParticles.size() > 1) {
        for (std::size_t i = 0; i < peripheralParticles.size(); ++i) {
            Particule* current = peripheralParticles[i];
            Particule* next = peripheralParticles[(i + 1) % peripheralParticles.size()];
            connectSpring(current, next, kOuterSpringStiffness);
        }
    }

}

void Blob::applyForces(float dt, bool useVerletIntegration, bool applyGravity, bool applyFriction, bool applySprings)
{
    if (applyGravity) {
        for (auto& particle : particles) {
            // Reporter l'application de la gravité au registre afin de réutiliser la même instance de générateur.
            registry.add(particle.get(), gravity.get());
        }
    }

    if (applyFriction) {
        for (auto& particle : particles) {
            // La force de frottement est également ajoutée au registre afin de conserver la cohérence.
            registry.add(particle.get(), friction.get());
        }
    }

    if (applySprings) {
        for (const SpringBinding& binding : springBindings) {
            if (!binding.owner || !binding.other)
                continue;
            // Ignorer les ressorts qui impliquent une particule détachée.
            if (detachedParticles.find(binding.owner) != detachedParticles.end())
                continue;
            if (detachedParticles.find(binding.other) != detachedParticles.end())
                continue;

            registry.add(binding.owner, binding.generator);
        }
    }

    if (!particles.empty()) {
        // Appliquer toutes les forces mises en attente en une seule passe, puis vider le registre pour l'image suivante.
        registry.updateForces(dt);
        registry.clear();
    }

    for (auto& particle : particles) {
        if (useVerletIntegration) {
            // L'intégration de Verlet offre une meilleure stabilité pour les systèmes à ressorts.
            particle->integrerVerlet(dt);
        } else {
            particle->integrerEuler(dt);
        }
    }
}

void Blob::detectAndResolveCollisions()
{
    collisionSystem.clear();
    collidingParticles.clear();

    for (std::size_t i = 0; i < particles.size(); ++i) {
        Particule* a = particles[i].get();
        for (std::size_t j = i + 1; j < particles.size(); ++j) {
            Particule* b = particles[j].get();
            if (SystemCollisionDetection::IsColliding(a, b)) {
                // Enregistrer les contacts sphère-sphère avec un léger rebond pour que le blob rebondisse.
                collisionSystem.add(a, b, 0.25f, CollisionType::Contact);
                collidingParticles.insert(a);
                collidingParticles.insert(b);
            }
        }
    }

    for (auto& particle : particles) {
        Particule* p = particle.get();

        if (p->_pos.y > playArea.getBottom()) {
            // Repousser les particules à l'intérieur de l'aire de jeu grâce à des plans virtuels.
            collisionSystem.addPlane(p, Vector3D(0.f, -1.f, 0.f), p->_pos.y - playArea.getBottom(), 0.25f, CollisionType::Resting);
            collidingParticles.insert(p);
        }

        if (p->_pos.y < playArea.getTop()) {
            collisionSystem.addPlane(p, Vector3D(0.f, 1.f, 0.f), playArea.getTop() - p->_pos.y, 0.05f, CollisionType::Contact);
            collidingParticles.insert(p);
        }

        if (p->_pos.x < playArea.getLeft()) {
            collisionSystem.addPlane(p, Vector3D(1.f, 0.f, 0.f), playArea.getLeft() - p->_pos.x, 0.2f, CollisionType::Contact);
            collidingParticles.insert(p);
        }

        if (p->_pos.x > playArea.getRight()) {
            collisionSystem.addPlane(p, Vector3D(-1.f, 0.f, 0.f), p->_pos.x - playArea.getRight(), 0.2f, CollisionType::Contact);
            collidingParticles.insert(p);
        }
    }

    // Résoudre tous les contacts et contraintes accumulés en une seule passe.
    collisionSystem.resolveAll();
}

void Blob::update(float dt, bool useVerletIntegration, bool applyGravity, bool applyFriction, bool applySprings)
{
    if (particles.empty())
        return;

    applyForces(dt, useVerletIntegration, applyGravity, applyFriction, applySprings);
    detectAndResolveCollisions();
}

void Blob::draw(bool showSprings, bool highlightCollisions) const
{
    ofPushStyle();
    ofNoFill();
    ofSetColor(60, 60, 60);
    ofDrawRectangle(playArea);
    ofPopStyle();

    if (showSprings) {
        ofSetColor(120, 160, 255, 160);
        for (const SpringConnection& connection : springConnections) {
            if (!connection.a || !connection.b)
                continue;
            if (detachedParticles.find(connection.a) != detachedParticles.end())
                continue;
            if (detachedParticles.find(connection.b) != detachedParticles.end())
                continue;
            ofDrawLine(connection.a->_pos.x, connection.a->_pos.y, connection.b->_pos.x, connection.b->_pos.y);
        }
    }

    for (const auto& particle : particles) {
        const Particule* p = particle.get();
        bool colliding = collidingParticles.find(p) != collidingParticles.end();

        ofColor color = ofColor::fromHex(0x4FC3F7);
        if (detachedParticles.find(p) != detachedParticles.end()) {
            color = ofColor::fromHex(0xFFD54F);
        }
        if (highlightCollisions && colliding) {
            color = ofColor::fromHex(0xFF7043);
        }

        ofSetColor(color);
        ofDrawCircle(p->_pos.x, p->_pos.y, p->rayonCollision);

        ofSetColor(255);
        std::string label = "m=" + ofToString(p->masse, 2);
        ofDrawBitmapStringHighlight(label, p->_pos.x + p->rayonCollision + 4.f, p->_pos.y - p->rayonCollision);
    }
}

std::size_t Blob::particleCount() const
{
    return particles.size();
}

std::size_t Blob::activeCollisionCount() const
{
    return collidingParticles.size();
}

void Blob::applyMovement(const Vector3D& inputDirection, float dt)
{
    if (!centerParticle || dt <= 0.f)
        return;

    auto hasInput = [](const Vector3D& vector) {
        return std::fabs(vector.x) > std::numeric_limits<float>::epsilon() ||
               std::fabs(vector.y) > std::numeric_limits<float>::epsilon() ||
               std::fabs(vector.z) > std::numeric_limits<float>::epsilon();
    };

    if (hasInput(inputDirection)) {
        float magnitude = std::sqrt(
                inputDirection.x * inputDirection.x +
                inputDirection.y * inputDirection.y +
                inputDirection.z * inputDirection.z);

        if (magnitude <= std::numeric_limits<float>::epsilon())
            return;

        Vector3D direction = inputDirection.scalar(1.f / magnitude);

        float safeDt = std::max(dt, 0.0001f);
        float impulse = kMovementImpulse * centerParticle->masse;
        Vector3D force = direction.scalar(impulse / safeDt);
        // Appliquer une force équivalente à une impulsion pour que la particule centrale accélère en douceur.
        centerParticle->addForce(force);
    } else {
        Vector3D brakeForce = centerParticle->_vel.scalar(-kMovementBrake * centerParticle->masse);
        // Appliquer un freinage doux en l'absence d'entrée pour garder le blob contrôlable.
        centerParticle->addForce(brakeForce);
    }
}

bool Blob::detachPeripheralParticle()
{
    if (particles.size() <= 1)
        return false;

    for (const auto& particle : particles) {
        Particule* p = particle.get();
        if (p == centerParticle)
            continue;
        if (detachedParticles.find(p) != detachedParticles.end())
            continue;

        // Enregistrer la particule comme détachée afin que les ressorts et l'affichage l'ignorent.
        detachedParticles.insert(p);
        return true;
    }

    return false;
}

void Blob::reattachAllParticles()
{
    detachedParticles.clear();
}

float Blob::totalMass() const
{
    float sum = 0.f;
    for (const auto& particle : particles) {
        sum += particle->masse;
    }
    return sum;
}
