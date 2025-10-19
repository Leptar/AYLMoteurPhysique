#include "Blob.h"

#include "3DVector.h"
#include "ofGraphics.h"
#include "ofUtils.h"

#include <algorithm>
#include <cmath>

namespace {
constexpr int kRows = 3;
constexpr int kCols = 3;
constexpr float kSpringStiffness = 25.f;
constexpr float kBungeeStiffness = 18.f;
constexpr float kRestThreshold = 1.0f;
}

Blob::Blob()
{
    gravity = std::make_unique<ForceGravity>();
    friction = std::make_unique<ForceFriction>();
}

void Blob::setup(const ofRectangle& bounds)
{
    buildBlob(bounds);
}

void Blob::reset(const ofRectangle& bounds)
{
    particles.clear();
    springGenerators.clear();
    springBindings.clear();
    springConnections.clear();
    cableConstraints.clear();
    rodConstraints.clear();
    collidingParticles.clear();
    collisionSystem.clear();
    registry.clear();
    buildBlob(bounds);
}

void Blob::setBounds(const ofRectangle& bounds)
{
    playArea = bounds;
}

void Blob::buildBlob(const ofRectangle& bounds)
{
    playArea = bounds;

    float horizontalSpacing = bounds.getWidth() / static_cast<float>(kCols + 1);
    float verticalSpacing = bounds.getHeight() / static_cast<float>(kRows + 1);
    float spacing = std::min(horizontalSpacing, verticalSpacing);

    Vector3D start(bounds.getLeft() + spacing, bounds.getTop() + spacing, 0.f);

    for (int row = 0; row < kRows; ++row) {
        for (int col = 0; col < kCols; ++col) {
            Vector3D position(start.x + col * spacing, start.y + row * spacing, 0.f);
            float mass = 0.8f + 0.2f * static_cast<float>(row + 1);
            float radius = 14.f;
            float linear = 0.4f;
            float quadratic = 0.02f;
            particles.emplace_back(std::make_unique<Particule>(position, Vector3D(0, 0, 0), Vector3D(0, 0, 0), mass, linear, quadratic, radius));
        }
    }

    auto index = [](int row, int col) { return row * kCols + col; };

    for (int row = 0; row < kRows; ++row) {
        for (int col = 0; col < kCols; ++col) {
            Particule* current = particles[index(row, col)].get();

            if (col + 1 < kCols) {
                Particule* neighbour = particles[index(row, col + 1)].get();
                float restLength = (neighbour->_pos - current->_pos).GetNorm();
                springConnections.push_back({current, neighbour, restLength});

                auto spring = std::make_unique<ForceRessortParticule>(neighbour, kSpringStiffness, restLength);
                springBindings.push_back({current, spring.get()});
                springGenerators.push_back(std::move(spring));

                auto springBack = std::make_unique<ForceRessortParticule>(current, kSpringStiffness, restLength);
                springBindings.push_back({neighbour, springBack.get()});
                springGenerators.push_back(std::move(springBack));

                cableConstraints.push_back({current, neighbour, restLength * 1.6f, 0.15f});
            }

            if (row + 1 < kRows) {
                Particule* neighbour = particles[index(row + 1, col)].get();
                float restLength = (neighbour->_pos - current->_pos).GetNorm();
                springConnections.push_back({current, neighbour, restLength});

                auto spring = std::make_unique<ForceRessortParticule>(neighbour, kSpringStiffness, restLength);
                springBindings.push_back({current, spring.get()});
                springGenerators.push_back(std::move(spring));

                auto springBack = std::make_unique<ForceRessortParticule>(current, kSpringStiffness, restLength);
                springBindings.push_back({neighbour, springBack.get()});
                springGenerators.push_back(std::move(springBack));

                cableConstraints.push_back({current, neighbour, restLength * 1.6f, 0.15f});
            }

            if (row + 1 < kRows && col + 1 < kCols) {
                Particule* diagonal = particles[index(row + 1, col + 1)].get();
                float restLength = (diagonal->_pos - current->_pos).GetNorm();
                rodConstraints.push_back({current, diagonal, restLength});
            }

            if (row + 1 < kRows && col - 1 >= 0) {
                Particule* diagonal = particles[index(row + 1, col - 1)].get();
                float restLength = (diagonal->_pos - current->_pos).GetNorm();
                rodConstraints.push_back({current, diagonal, restLength});
            }
        }
    }

    Particule* topCenter = particles[index(0, kCols / 2)].get();
    Vector3D anchor(bounds.getCenter().x, bounds.getTop() - spacing * 0.5f, 0.f);
    auto fixedSpring = std::make_unique<ForceRessortFixe>(anchor, kSpringStiffness, spacing);
    springBindings.push_back({topCenter, fixedSpring.get()});
    springGenerators.push_back(std::move(fixedSpring));

    Particule* bottomCenter = particles[index(kRows - 1, kCols / 2)].get();
    Vector3D anchorBottom(bounds.getCenter().x, bounds.getBottom() + spacing * 0.25f, 0.f);
    auto lowerSpring = std::make_unique<ForceRessortFixe>(anchorBottom, kBungeeStiffness, spacing);
    springBindings.push_back({bottomCenter, lowerSpring.get()});
    springGenerators.push_back(std::move(lowerSpring));

    updatePotentialEnergy();
}

void Blob::applyForces(float dt, bool useVerletIntegration, bool applyGravity, bool applyFriction, bool applySprings)
{
    if (applyGravity) {
        for (auto& particle : particles) {
            registry.add(particle.get(), gravity.get());
        }
    }

    if (applyFriction) {
        for (auto& particle : particles) {
            registry.add(particle.get(), friction.get());
        }
    }

    if (applySprings) {
        for (const SpringBinding& binding : springBindings) {
            registry.add(binding.owner, binding.generator);
        }
    }

    if (!particles.empty()) {
        registry.updateForces(dt);
        registry.clear();
    }

    for (auto& particle : particles) {
        if (useVerletIntegration) {
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
                collisionSystem.add(a, b, 0.25f, CollisionType::Contact);
                collidingParticles.insert(a);
                collidingParticles.insert(b);
            }
        }
    }

    for (const CableConstraint& cable : cableConstraints) {
        float currentLength = (cable.a->_pos - cable.b->_pos).GetNorm();
        if (currentLength > cable.maxLength) {
            collisionSystem.addCableConstraint(cable.a, cable.b, cable.maxLength, cable.restitution);
            collidingParticles.insert(cable.a);
            collidingParticles.insert(cable.b);
        }
    }

    for (const RodConstraint& rod : rodConstraints) {
        float currentLength = (rod.a->_pos - rod.b->_pos).GetNorm();
        if (std::fabs(currentLength - rod.length) > kRestThreshold) {
            collisionSystem.addRodConstraint(rod.a, rod.b, rod.length);
            collidingParticles.insert(rod.a);
            collidingParticles.insert(rod.b);
        }
    }

    for (auto& particle : particles) {
        Particule* p = particle.get();

        if (p->_pos.y > playArea.getBottom()) {
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

    collisionSystem.resolveAll();
}

void Blob::updatePotentialEnergy()
{
    float g = std::fabs(gravity->Gravity.y);
    float floorY = playArea.getBottom();

    cachedPotentialEnergy = 0.f;
    for (const auto& particle : particles) {
        float height = std::max(0.f, floorY - particle->_pos.y);
        cachedPotentialEnergy += particle->masse * g * height;
    }
}

void Blob::update(float dt, bool useVerletIntegration, bool applyGravity, bool applyFriction, bool applySprings)
{
    if (particles.empty())
        return;

    applyForces(dt, useVerletIntegration, applyGravity, applyFriction, applySprings);
    detectAndResolveCollisions();
    updatePotentialEnergy();
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
            ofDrawLine(connection.a->_pos.x, connection.a->_pos.y, connection.b->_pos.x, connection.b->_pos.y);
        }
    }

    for (const auto& particle : particles) {
        const Particule* p = particle.get();
        bool colliding = collidingParticles.find(p) != collidingParticles.end();

        ofColor color = ofColor::fromHex(0x4FC3F7);
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

float Blob::totalMass() const
{
    float sum = 0.f;
    for (const auto& particle : particles) {
        sum += particle->masse;
    }
    return sum;
}

float Blob::potentialEnergy() const
{
    return cachedPotentialEnergy;
}
