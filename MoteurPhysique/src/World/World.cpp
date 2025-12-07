#include "World.h"

#include <algorithm>

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
	m_rigidBodyCollisionSystem = std::make_unique<SystemeCollisionDetection>();

	// Initialiser les limites du monde pour l'Octree.
	float worldSize = 500.0f;
	m_worldBounds = AABB(Vector3D(-worldSize, -worldSize, -worldSize), Vector3D(worldSize, worldSize, worldSize));
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

SystemCollisionDetection* World::getParticleCollisionDetector()
{
    return &m_particleCollisionDetector;
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

    // Créer la primitive Box avec les bonnes dimensions et l'associer au corps rigide.
    auto primitiveBox = std::make_unique<Box>(halfExtents);
    box.primitive = std::move(primitiveBox);

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

void World::addStaticPlane(const Vector3D& normal, float offset)
{
    auto plane = std::make_unique<Plane>();
    plane->normal = normal;
    plane->PlaneOffset = offset;
    m_staticGeometry.push_back(std::move(plane));
}

void World::broadPhaseDetection(std::vector<RigidBodyBox>& rigidBodies) {
	// Construire l'octree
	m_octree = std::make_unique<Octree>(m_worldBounds);

	// MAJ AABB et insert dans le octree
	for (auto & body_box : rigidBodies) {
		body_box.body.calculateWorldAABB(*body_box.primitive);
		m_octree->insert(body_box.primitive.get(), body_box.body.worldAABB);
	}

	// Genere les paires potentielles en parcourant l'Octree
	std::vector<std::pair<Primitive*, Primitive*>> potentialCollisions;
	m_octree->generatePotentialCollisions(potentialCollisions);

	// La méthode ci-dessus peut générer des doublons si un objet est retourné plusieurs fois.
	// On trie et on supprime les doublons pour s'assurer que chaque paire est unique.
	std::sort(potentialCollisions.begin(), potentialCollisions.end());
	potentialCollisions.erase(std::unique(potentialCollisions.begin(), potentialCollisions.end()), potentialCollisions.end());

	// À ce stade, normalement `potentialCollisions` contient toutes les paires à tester en phase restreinte.
	narrowPhaseDetection(potentialCollisions, rigidBodies);
	
	std::cout << "Collisions potentielles cette frame: " << potentialCollisions.size() << std::endl;
}

void World::narrowPhaseDetection(const std::vector<std::pair<Primitive *, Primitive *>> & potentialCollisions, std::vector<RigidBodyBox>& rigidBodies)
{
    // 1. On vide les collisions de la frame précédente
    if (m_rigidBodyCollisionSystem) {
        m_rigidBodyCollisionSystem->clear();
    } else {
        return;
    }

    // 2. On parcourt toutes les paires dynamiques (Boîte-Boîte) renvoyées par la Broad Phase
    for (const auto& pair : potentialCollisions)
    {
        Primitive* p1 = pair.first;
        Primitive* p2 = pair.second;

        if (!p1 || !p2) continue;

        // Pour l'instant, on ne gère que Boîte-Boîte (qui sera implémenté plus tard)
        Box* box1 = dynamic_cast<Box*>(p1);
        Box* box2 = dynamic_cast<Box*>(p2);

        if (box1 && box2) {
            m_rigidBodyCollisionSystem->DetectBoxBox(box1, box2);
        }
    }

    // 3. On teste chaque objet dynamique contre la géométrie statique (les plans)
    for (auto& body_box : rigidBodies)
    {
        Box* box = dynamic_cast<Box*>(body_box.primitive.get());
        if (!box) continue;

        for (const auto& staticPrimitive : m_staticGeometry)
        {
            Plane* plane = dynamic_cast<Plane*>(staticPrimitive.get());
            if (plane)
            {
                m_rigidBodyCollisionSystem->DetectBoxPlane(box, plane);
            }
        }
    }
}

void World::update(float deltaTime, std::vector<RigidBodyBox>& rigidBodies)
{
	// Particule
    m_forceRegistry.updateForces(deltaTime);

    for (Particule* p : m_particules)
    {
        p->integrerVerlet(deltaTime);
        p->clearForce();
    }

    m_particleCollisionDetector.resolveAll(); // Résolution pour les particules

	// --- Simulation des Corps Rigides ---

	// 1. Appliquer les forces et intégrer le mouvement
	for (auto& bodybox : rigidBodies) {
		applyRigidBodyForces(bodybox.body, deltaTime);
		bodybox.body.integrer(deltaTime);
	}

	// 2. Détecter les collisions (Phases élargie et restreinte)
    broadPhaseDetection(rigidBodies);

	// 3. Résoudre les collisions
    m_rigidBodyCollisionSystem->resolveAll();
}

void World::drawOctree() const
{
    if (m_octree)
    {
        m_octree->draw();
    }
}
