#include "World.h"

#include <algorithm>
#include <array>

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

    // Synchroniser la primitive de collision avec ce corps rigide
    if (Box* primitiveBox = dynamic_cast<Box*>(box.primitive)) {
        primitiveBox->HalfExtent = halfExtents;
    } else {
        delete box.primitive;
        box.primitive = new Box(halfExtents);
    }
    box.primitive->corpsRigide = &box.body;
    box.body.calculateWorldAABB(*box.primitive);

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

void World::stepRigidBodies(std::vector<RigidBodyBox>& bodies, float deltaTime, const AABB& worldBounds)
{
    if (deltaTime <= 0.f)
    {
        return;
    }

    // Actualise les limites puis reconstruit les plans statiques attachés.
    m_worldBounds = worldBounds;
    buildBoundaryPlanes();

    // Référence les corps actifs pour la phase de collision.
    m_rigidBodies.clear();
    for (auto& box : bodies)
    {
        if (!box.primitive)
        {
            continue;
        }
        box.primitive->corpsRigide = &box.body;
        m_rigidBodies.push_back(&box);
    }

    // Intégration des corps rigides (forces -> vitesses -> positions).
    for (RigidBodyBox* boxPtr : m_rigidBodies)
    {
        applyRigidBodyForces(boxPtr->body, deltaTime);
        boxPtr->body.integrer(deltaTime);
        boxPtr->body.calculateWorldAABB(*boxPtr->primitive);
    }

    // Phase élargie : reconstruction de l'octree avec les sphères englobantes.
    broadPhaseDetection();

    // Phase restreinte : génération des contacts et résolution par impulsions.
    // Les paires potentielles sont calculées dans broadPhaseDetection().
}

void World::broadPhaseDetection() {
        // Construire l'octree
        m_octree = std::make_unique<Octree>(m_worldBounds);

        // MAJ AABB et insert dans le octree
        for (auto & body_box : m_rigidBodies) {
                body_box->body.calculateWorldAABB(*body_box->primitive);

                // Volume élargi : sphère englobante -> AABB pour l'octree.
                float radius = body_box->boundingRadius;
                Vector3D center = body_box->body.getPosition();
                AABB sphereBounds(
                    center - Vector3D(radius, radius, radius),
                    center + Vector3D(radius, radius, radius));

                m_octree->insert(body_box->primitive, sphereBounds);
        }

        // Genere les pairs potentielles
        std::vector<std::pair<Primitive*, Primitive*>> potentialCollisions;

        for (auto& body_boxA : m_rigidBodies) {
                Primitive* primitiveA = body_boxA->primitive;

                float radius = body_boxA->boundingRadius;
                Vector3D center = body_boxA->body.getPosition();
                AABB queryBounds(
                    center - Vector3D(radius, radius, radius),
                    center + Vector3D(radius, radius, radius));

                std::vector<Primitive*> condidates = m_octree->request(queryBounds);

                for (Primitive* primitiveB : condidates) {
                        if (primitiveA < primitiveB) {
                                potentialCollisions.push_back(std::make_pair(primitiveA, primitiveB));
                        }
                }
        }


        // À ce stade, normalement `potentialCollisions` contient toutes les paires à tester en phase restreinte.
        narrowPhaseDetection(potentialCollisions);
	
	std::cout << "Collisions potentielles cette frame: " << potentialCollisions.size() << std::endl;

}

void World::narrowPhaseDetection(const std::vector<std::pair<Primitive *, Primitive *>> & potentialCollisions)
{
    // 1. On vide les collisions de la frame précédente
    if (collisionSystem) {
        collisionSystem->clear();
    } else {
        return;
    }

    // 2. On parcourt toutes les paires renvoyées par la Broad Phase
    for (const auto& pair : potentialCollisions)
    {
        Primitive* p1 = pair.first;
        Primitive* p2 = pair.second;

        if (!p1 || !p2) continue;

        // 3. Identification des types (RTTI)
        // On essaie de caster p1 et p2 en Box ou Plane
        Box* box1 = dynamic_cast<Box*>(p1);
        Box* box2 = dynamic_cast<Box*>(p2);
        Plane* plane1 = dynamic_cast<Plane*>(p1);
        Plane* plane2 = dynamic_cast<Plane*>(p2);

        // --- Cas : Boîte vs Plan ---
        if (box1 && plane2)
        {
            collisionSystem->DetectBoxPlane(box1, plane2);
        }
        else if (plane1 && box2)
        {
            // On inverse les arguments car DetectBoxPlane attend (Box, Plane)
            collisionSystem->DetectBoxPlane(box2, plane1);
        }

        // --- Cas : Boîte vs Boîte (approximation AABB monde) ---
        if (box1 && box2)
        {
            const AABB& aabbA = box1->corpsRigide->worldAABB;
            const AABB& aabbB = box2->corpsRigide->worldAABB;

            if (aabbA.intersects(aabbB))
            {
                // Direction de séparation approximée : du centre de A vers B.
                Vector3D centerA = aabbA.getCenter();
                Vector3D centerB = aabbB.getCenter();
                Vector3D normal = (centerB - centerA).normalize();
                if (normal.GetNorm() < 1e-3f)
                {
                    normal = Vector3D(0.f, 1.f, 0.f);
                }

                // Profondeur minimale selon les axes du monde.
                Vector3D overlap(
                    std::min(aabbA.max.x, aabbB.max.x) - std::max(aabbA.min.x, aabbB.min.x),
                    std::min(aabbA.max.y, aabbB.max.y) - std::max(aabbA.min.y, aabbB.min.y),
                    std::min(aabbA.max.z, aabbB.max.z) - std::max(aabbA.min.z, aabbB.min.z));

                float penetration = std::min({ overlap.x, overlap.y, overlap.z });
                Vector3D contactPoint = (centerA + centerB).scalar(0.5f);

                collisionSystem->add(box1, box2, contactPoint, normal, penetration, 0.4f, collision_type::Contact);
            }
        }
    }

    // 3. Boîte vs plans statiques définis par le volume englobant.
    if (collisionSystem)
    {
        for (auto& boxPtr : m_rigidBodies)
        {
            Box* box = dynamic_cast<Box*>(boxPtr->primitive);
            if (!box)
            {
                continue;
            }

            for (const auto& plane : m_boundaryPlanes)
            {
                collisionSystem->DetectBoxPlane(box, plane.get());
            }
        }
    }

    if (collisionSystem)
    {
        collisionSystem->resolveAll();
    }

    // Les corrections de position peuvent invalider les AABB monde : on les remet à jour.
    for (auto* bodyBox : m_rigidBodies)
    {
        bodyBox->body.calculateWorldAABB(*bodyBox->primitive);
    }
}

void World::buildBoundaryPlanes()
{
    // Six plans alignés sur les faces du volume englobant.
    if (m_boundaryPlanes.size() != 6)
    {
        m_boundaryPlanes.clear();
        m_boundaryBodies.clear();
        m_boundaryPlanes.resize(6);
        m_boundaryBodies.resize(6);

        for (int i = 0; i < 6; ++i)
        {
            m_boundaryPlanes[i] = std::make_unique<Plane>();
            m_boundaryBodies[i] = std::make_unique<CorpsRigide>();
            m_boundaryBodies[i]->setInverseMasse(0.f); // immobile
            m_boundaryPlanes[i]->corpsRigide = m_boundaryBodies[i].get();
        }
    }

    // Normales dirigées vers l'intérieur du volume englobant.
    // 0: face min X, 1: face max X, 2: face min Y, 3: face max Y, 4: face min Z, 5: face max Z
    std::array<Vector3D, 6> normals = {
        Vector3D(1.f, 0.f, 0.f),
        Vector3D(-1.f, 0.f, 0.f),
        Vector3D(0.f, 1.f, 0.f),
        Vector3D(0.f, -1.f, 0.f),
        Vector3D(0.f, 0.f, 1.f),
        Vector3D(0.f, 0.f, -1.f)
    };

    std::array<float, 6> offsets = {
        m_worldBounds.min.x,
        -m_worldBounds.max.x,
        m_worldBounds.min.y,
        -m_worldBounds.max.y,
        m_worldBounds.min.z,
        -m_worldBounds.max.z
    };

    for (int i = 0; i < 6; ++i)
    {
        Plane* plane = m_boundaryPlanes[i].get();
        plane->normal = normals[i];
        plane->PlaneOffset = offsets[i];
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
