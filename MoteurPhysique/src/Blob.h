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

    /// Configure the blob inside the given bounds.
    void setup(const ofRectangle& bounds);
    /// Reset all particles and rebuild the blob inside the bounds.
    void reset(const ofRectangle& bounds);
    /// Update the playable area without rebuilding the particles.
    void setBounds(const ofRectangle& bounds);
    /// Integrate physics and handle collisions for the blob.
    void update(float dt, bool useVerletIntegration, bool applyGravity, bool applyFriction, bool applySprings);
    /// Draw the blob particles and optional spring visuals.
    void draw(bool showSprings, bool highlightCollisions) const;

    /// Apply player input forces to the central particle.
    void applyMovement(const Vector3D& inputDirection, float dt);
    /// Mark the next available peripheral particle as detached.
    bool detachPeripheralParticle();
    /// Reconnect all previously detached particles.
    void reattachAllParticles();

    /// Total number of particles currently composing the blob.
    std::size_t particleCount() const;
    /// Number of particles currently colliding.
    std::size_t activeCollisionCount() const;
    /// Sum of the mass of every particle in the blob.
    float totalMass() const;
    /// Cached gravitational potential energy of the blob.
    float potentialEnergy() const;

private:
    /// Helper structure remembering which generator acts on which particles.
    struct SpringBinding {
        Particule* owner = nullptr;
        Particule* other = nullptr;
        ParticuleForceGenerator* generator = nullptr;
    };

    /// Simplified representation of a spring for rendering purposes.
    struct SpringConnection {
        Particule* a = nullptr;
        Particule* b = nullptr;
        float restLength = 0.f;
    };

    /// Allocate particles in a ring around the controllable center particle.
    void buildBlob(const ofRectangle& bounds);
    /// Queue the requested forces and integrate the motion of each particle.
    void applyForces(float dt, bool useVerletIntegration, bool applyGravity, bool applyFriction, bool applySprings);
    /// Detect particle/plane overlaps and resolve them through the collision system.
    void detectAndResolveCollisions();
    /// Recompute the blob's gravitational potential energy cache.
    void updatePotentialEnergy();

    ofRectangle playArea;
    std::vector<std::unique_ptr<Particule>> particles;
    Particule* centerParticle = nullptr;
    ParticuleForceRegistry registry;
    std::vector<std::unique_ptr<ParticuleForceGenerator>> springGenerators;
    std::vector<SpringBinding> springBindings;
    std::vector<SpringConnection> springConnections;
    std::unordered_set<const Particule*> detachedParticles;

    std::unique_ptr<ForceGravity> gravity;
    std::unique_ptr<ForceFriction> friction;

    SystemCollisionDetection collisionSystem;
    std::unordered_set<const Particule*> collidingParticles;

    float cachedPotentialEnergy = 0.f;
};
