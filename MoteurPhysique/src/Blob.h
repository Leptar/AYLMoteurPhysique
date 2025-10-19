#pragma once

#include "ForceGenerator/ForceFriction.h"
#include "ForceGenerator/ForceGravity.h"
#include "ForceGenerator/ForceRessortFixe.h"
#include "ForceGenerator/ForceRessortParticule.h"
#include "ForceGenerator/ParticuleForceRegistry.h"
#include "SystemeCollision/SystemCollisionDetection.h"
#include "ofColor.h"
#include "ofRectangle.h"

#include <memory>
#include <unordered_set>
#include <utility>
#include <vector>

class Blob
{
public:
    Blob();

    void setup(const ofRectangle& bounds);
    void reset(const ofRectangle& bounds);
    void setBounds(const ofRectangle& bounds);
    void update(float dt, bool useVerletIntegration, bool applyGravity, bool applyFriction, bool applySprings);
    void draw(bool showSprings, bool highlightCollisions) const;

    std::size_t particleCount() const;
    std::size_t activeCollisionCount() const;
    float totalMass() const;
    float potentialEnergy() const;

private:
    struct SpringBinding {
        Particule* owner = nullptr;
        ParticuleForceGenerator* generator = nullptr;
    };

    struct SpringConnection {
        Particule* a = nullptr;
        Particule* b = nullptr;
        float restLength = 0.f;
    };

    struct CableConstraint {
        Particule* a = nullptr;
        Particule* b = nullptr;
        float maxLength = 0.f;
        float restitution = 0.2f;
    };

    struct RodConstraint {
        Particule* a = nullptr;
        Particule* b = nullptr;
        float length = 0.f;
    };

    void buildBlob(const ofRectangle& bounds);
    void applyForces(float dt, bool useVerletIntegration, bool applyGravity, bool applyFriction, bool applySprings);
    void detectAndResolveCollisions();
    void updatePotentialEnergy();

    ofRectangle playArea;
    std::vector<std::unique_ptr<Particule>> particles;
    ParticuleForceRegistry registry;
    std::vector<std::unique_ptr<ParticuleForceGenerator>> springGenerators;
    std::vector<SpringBinding> springBindings;
    std::vector<SpringConnection> springConnections;
    std::vector<CableConstraint> cableConstraints;
    std::vector<RodConstraint> rodConstraints;

    std::unique_ptr<ForceGravity> gravity;
    std::unique_ptr<ForceFriction> friction;

    SystemCollisionDetection collisionSystem;
    std::unordered_set<const Particule*> collidingParticles;

    float cachedPotentialEnergy = 0.f;
};
