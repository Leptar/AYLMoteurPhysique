#include "World.h"

#include <algorithm>
#include <cmath>
#include <set>
#include <unordered_map>

#include "ofMath.h"
#include "ofMathConstants.h"

#include "../MathStruct/Quaternion.h"

namespace {

Vector3D normalizedAxis(float x, float y, float z)
{
    Vector3D axis(x, y, z);
    float norm = axis.GetNorm();
    if (norm <= 1e-4f) {
        return Vector3D(0.f, 1.f, 0.f);
    }
    return axis.scalar(1.f / norm);
}

}

World::World()
{
	collisionSystem = std::make_unique<SystemeCollisionDetection>();
}

World::~World()
{
    for (Particule* p : m_particules)
    {
        delete p;
    }
    m_particules.clear();
}

void World::addParticule(Particule* particule)
{
    if (particule)
    {
        m_particules.push_back(particule);
    }
}

ParticuleForceRegistry* World::getForceRegistry()
{
    return &m_forceRegistry;
}

RigidBodyForceRegistry* World::getRigidBodyForceRegistry()
{
    return &m_rigidBodyForceRegistry;
}

const RigidBodyForceRegistry* World::getRigidBodyForceRegistry() const
{
    return &m_rigidBodyForceRegistry;
}

void World::applyRigidBodyForces(CorpsRigide& body, float deltaTime) const
{
    m_rigidBodyForceRegistry.updateForces(body, deltaTime);
}

SystemCollisionDetection* World::getCollisionDetector()
{
    return &m_collisionDetector;
}

RigidBodyBox World::createRigidBodyBox(const Vector3D& position,
                                       const Vector3D& halfExtents,
                                       float mass,
                                       const ofColor& color,
                                       const Vector3D& initialLinearVelocity,
                                       const Vector3D& initialAngularVelocity) const
{
    RigidBodyBox box;
    box.mass = std::max(mass, 0.1f);
    box.halfExtents = halfExtents;
    box.color = color;
    box.reachedGoal = false;
    box.outOfBounds = false;

    Vector3D radiusVec = halfExtents;
    box.boundingRadius = std::max(radiusVec.GetNorm(), 6.f);

    float invMass = (box.mass > 0.f) ? 1.f / box.mass : 0.f;
    box.body.setInverseMasse(invMass);

    float hx = std::max(halfExtents.x, 1.f);
    float hy = std::max(halfExtents.y, 1.f);
    float hz = std::max(halfExtents.z, 1.f);

    float Ixx = (box.mass / 12.f) * (hy * hy + hz * hz);
    float Iyy = (box.mass / 12.f) * (hx * hx + hz * hz);
    float Izz = (box.mass / 12.f) * (hx * hx + hy * hy);

    float invIxx = (Ixx > 0.f) ? 1.f / Ixx : 0.f;
    float invIyy = (Iyy > 0.f) ? 1.f / Iyy : 0.f;
    float invIzz = (Izz > 0.f) ? 1.f / Izz : 0.f;

    Matrix3 invIbody(
            invIxx, 0.f,    0.f,
            0.f,    invIyy, 0.f,
            0.f,    0.f,    invIzz);
    box.body.setInverseInertiaTensorBody(invIbody);

    Vector3D axis = normalizedAxis(ofRandom(-1.f, 1.f), ofRandom(-1.f, 1.f), ofRandom(-1.f, 1.f));
    float angle = ofRandom(0.f, TWO_PI);
    Quaternion orientation = Quaternion::FromAxisAngle(axis, angle);

    box.body.setOrientation(orientation);
    box.body.setPosition(position);
    box.body.setVelocite(initialLinearVelocity);
    box.body.setVelociteAngulaire(initialAngularVelocity);
    box.body.clearAccumulators();

    box.primitive = std::make_unique<Box>(halfExtents);
    box.primitive->corpsRigide = &box.body;

    return box;
}

std::vector<RigidBodyBox> World::createRigidBodyGame(int boxCount,
                                                     float dropperSpawnHeight,
                                                     float boundsX,
                                                     float boundsZ) const
{
    std::vector<RigidBodyBox> boxes;
    boxes.reserve(std::max(0, boxCount));

    for (int i = 0; i < boxCount; ++i) {
        Vector3D halfExtents(
                ofRandom(14.f, 24.f),
                ofRandom(12.f, 26.f),
                ofRandom(14.f, 24.f));
        Vector3D dimensions = halfExtents.scalar(2.f);
        float volume = dimensions.x * dimensions.y * dimensions.z;
        float density = 0.00085f;
        float mass = ofClamp(volume * density, 12.f, 48.f);

        Vector3D position(
                ofRandom(-boundsX * 0.7f, boundsX * 0.7f),
                dropperSpawnHeight + ofRandom(-40.f, 40.f),
                ofRandom(-boundsZ * 0.7f, boundsZ * 0.7f));

        ofColor color = ofColor::fromHsb(ofRandom(30, 200), 200, 235);

        Vector3D initialVel(
                ofRandom(-45.f, 45.f),
                ofRandom(-25.f, 25.f),
                ofRandom(-45.f, 45.f));
        Vector3D angularVel(
                ofRandom(-1.8f, 1.8f),
                ofRandom(-1.8f, 1.8f),
                ofRandom(-1.8f, 1.8f));

        boxes.push_back(createRigidBodyBox(position, halfExtents, mass, color, initialVel, angularVel));
    }

    return boxes;
}

void World::broadPhaseDetection() {
        if (!collisionSystem)
        {
                return;
        }

        // Construire l'octree à partir des limites actuelles
        m_octree = std::make_unique<Octree>(m_worldBounds);
        m_octreeDebugNodes.clear();

        std::unordered_map<Primitive*, AABB> sphereBounds;

        for (auto & body_box : m_rigidBodies) {
                if (!body_box->primitive)
                {
                        continue;
                }

                body_box->body.calculateWorldAABB(*body_box->primitive);

                // Volume englobant sphérique pour la broad phase
                Vector3D center = body_box->body.getPosition();
                float radius = body_box->boundingRadius;
                Vector3D min = center - Vector3D(radius, radius, radius);
                Vector3D max = center + Vector3D(radius, radius, radius);
                AABB bound(min, max);

                sphereBounds[body_box->primitive.get()] = bound;
                m_octree->insert(body_box->primitive.get(), bound);
        }

        // Conserve les noeuds pour l'affichage debug
        if (m_octree)
        {
                m_octree->gatherNodes(m_octreeDebugNodes);
        }

        // Génère les paires potentielles
        std::vector<std::pair<Primitive*, Primitive*>> potentialCollisions;

        for (auto& body_boxA : m_rigidBodies) {
                Primitive* primitiveA = body_boxA->primitive.get();
                if (!primitiveA)
                {
                        continue;
                }

                std::vector<Primitive*> condidates = m_octree->request(sphereBounds[primitiveA]);

                for (Primitive* primitiveB : condidates) {
                        if (primitiveA < primitiveB) {
                                potentialCollisions.push_back(std::make_pair(primitiveA, primitiveB));
                        }
                }
        }

        // À ce stade, normalement `potentialCollisions` contient toutes les paires à tester en phase restreinte.
        narrowPhaseDetection(potentialCollisions);
}

void World::narrowPhaseDetection(const std::vector<std::pair<Primitive *, Primitive *>> & potentialCollisions)
{
    if (!collisionSystem)
    {
        return;
    }

    collisionSystem->clear();

    for (const auto& pair : potentialCollisions)
    {
        Primitive* p1 = pair.first;
        Primitive* p2 = pair.second;

        if (!p1 || !p2) continue;

        Box* box1 = dynamic_cast<Box*>(p1);
        Box* box2 = dynamic_cast<Box*>(p2);
        Plane* plane1 = dynamic_cast<Plane*>(p1);
        Plane* plane2 = dynamic_cast<Plane*>(p2);

        if (box1 && box2)
        {
            detectSphereSphere(box1, box2);
        }
        else if (box1 && plane2)
        {
            collisionSystem->DetectBoxPlane(box1, plane2);
        }
        else if (plane1 && box2)
        {
            collisionSystem->DetectBoxPlane(box2, plane1);
        }
    }
}

void World::detectSphereSphere(Box* a, Box* b)
{
    if (!collisionSystem || !a || !b || !a->corpsRigide || !b->corpsRigide)
    {
        return;
    }

    Vector3D posA = a->corpsRigide->getPosition();
    Vector3D posB = b->corpsRigide->getPosition();
    Vector3D delta = posB - posA;

    float distanceSq = delta.dot(delta);
    float radiusA = std::max(a->HalfExtent.GetNorm(), 1.f);
    float radiusB = std::max(b->HalfExtent.GetNorm(), 1.f);
    float radiusSum = radiusA + radiusB;

    if (distanceSq > radiusSum * radiusSum)
    {
        return;
    }

    float distance = std::sqrt(std::max(distanceSq, 1e-6f));
    Vector3D normal = (distance > 1e-4f) ? delta.scalar(1.f / distance) : Vector3D(0.f, 1.f, 0.f);
    float penetration = radiusSum - distance;

    Vector3D contactPoint = posA + normal.scalar(radiusA - penetration * 0.5f);

    collisionSystem->add(a, b, contactPoint, normal, penetration, 0.45f, collision_type::Contact);
}

void World::stepRigidBodies(std::vector<RigidBodyBox>& rigidBodies,
                            const std::vector<Plane>& staticPlanes,
                            float deltaTime,
                            const AABB& worldBounds)
{
    m_worldBounds = worldBounds;
    m_rigidBodies.clear();

    for (auto& box : rigidBodies)
    {
        if (!box.primitive)
        {
            box.primitive = std::make_unique<Box>(box.halfExtents);
        }

        box.primitive->HalfExtent = box.halfExtents;
        box.primitive->corpsRigide = &box.body;
        m_rigidBodies.push_back(&box);
    }

    for (auto& entry : m_rigidBodies)
    {
        applyRigidBodyForces(entry->body, deltaTime);
        entry->body.integrer(deltaTime);
    }

    // Broad puis narrow phase pour les collisions dynamique-dynamique
    broadPhaseDetection();

    // Détection Box-Plane en phase restreinte (plans statiques)
    if (collisionSystem)
    {
        for (auto& entry : m_rigidBodies)
        {
            for (const Plane& planeTemplate : staticPlanes)
            {
                Plane plane = planeTemplate;
                collisionSystem->DetectBoxPlane(entry->primitive.get(), &plane);
            }
        }

        collisionSystem->resolveAll();
    }

    for (auto& entry : m_rigidBodies)
    {
        entry->body.clearAccumulators();
    }
}

void World::update(float deltaTime)
{
	// Particule
    m_forceRegistry.updateForces(deltaTime);

    for (Particule* p : m_particules)
    {
        p->integrerVerlet(deltaTime);
        p->clearForce();
    }

    m_collisionDetector.resolveAll();

	// CorpsRigide
	for (auto& bodybox : m_rigidBodies) {
		applyRigidBodyForces(bodybox->body, deltaTime);

		bodybox->body.integrer(deltaTime);
	}

	broadPhaseDetection();
	// TODO : phase restreinte et resolution

	for (auto& bodybox : m_rigidBodies) {
		bodybox->body.clearAccumulators();
	}

}
