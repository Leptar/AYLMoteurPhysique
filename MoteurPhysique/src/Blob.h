#pragma once

#include "3DVector.h"
#include "ForceGenerator/ForceFriction.h"
#include "ForceGenerator/ForceGravity.h"
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

    void nudge(const Vector3D& impulse, float intensity = 1.f);

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

    void buildBlob(const ofRectangle& bounds);
    void applyForces(float dt, bool useVerletIntegration, bool applyGravity, bool applyFriction, bool applySprings);
    void detectAndResolveCollisions();
    void updatePotentialEnergy();

    ofRectangle playArea;
    std::vector<std::unique_ptr<Particule>> particles;
    Particule* centerParticle = nullptr;
    ParticuleForceRegistry registry;
    std::vector<std::unique_ptr<ParticuleForceGenerator>> springGenerators;
    std::vector<SpringBinding> springBindings;
    std::vector<SpringConnection> springConnections;

    std::unique_ptr<ForceGravity> gravity;
    std::unique_ptr<ForceFriction> friction;

    SystemCollisionDetection collisionSystem;
    std::unordered_set<const Particule*> collidingParticles;

    float cachedPotentialEnergy = 0.f;
};
