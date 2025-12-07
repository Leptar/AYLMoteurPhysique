#pragma once

#include "Octree.h"

#include <vector>
#include "../MathStruct/3DVector.h"
#include "../particule.h"
#include "../ForceGenerator/ParticuleForceRegistry.h"
#include "../ForceGenerator/RigidBodyForceRegistry.h"
#include "RigidBodyBox.h"
#include "SystemeCollisionDetection.h"

class World
{
public:
    World();
    ~World();

    /**
     * @brief
     * Appelle tous les systèmes dans l'ordre (Forces, Intégration, Collisions) pour l'exécuter à chaque frame.
     */
    void update(float deltaTime, std::vector<RigidBodyBox>& rigidBodies);

    /**
     * @brief Ajoute une particule à la simulation.
     */
    void addParticule(Particule* particule);

    /**
     * @brief Donne accès au registre des forces pour y lier des générateurs.
     */
    ParticuleForceRegistry* getForceRegistry();

    /**
     * @brief Donne accès au registre des forces appliquées aux corps rigides.
     */
    RigidBodyForceRegistry* getRigidBodyForceRegistry();
    const RigidBodyForceRegistry* getRigidBodyForceRegistry() const;

    /**
     * @brief Applique toutes les forces enregistrées au corps rigide donné.
     */
    void applyRigidBodyForces(CorpsRigide& body, float deltaTime) const;


    /**
     * @brief Crée et configure un pavé rigide prêt à être simulé.
     */
    RigidBodyBox createRigidBodyBox(const Vector3D& position,
                                    const Vector3D& halfExtents,
                                    float mass,
                                    const ofColor& color,
                                    const Vector3D& initialLinearVelocity = Vector3D(),
                                    const Vector3D& initialAngularVelocity = Vector3D()) const;

    /**
     * @brief Génère une collection de pavés rigides aléatoires pour initialiser la scène du jeu.
     */
    std::vector<RigidBodyBox> createRigidBodyGame(int boxCount,
                                                  float dropperSpawnHeight,
                                                  float boundsX,
                                                  float boundsZ) const;

	void drawOctree() const;
private:
    std::vector<Particule*> m_particules;

    ParticuleForceRegistry m_forceRegistry;
    RigidBodyForceRegistry m_rigidBodyForceRegistry;

	// Systeme de collision
	std::unique_ptr<SystemeCollisionDetection> collisionSystem;
	
	// L'Octree, reconstruit à chaque frame
	std::unique_ptr<Octree> m_octree;

	// Les limites de l'espace de simulation
	AABB m_worldBounds;

	// Méthodes privées pour la détection de collision
	void broadPhaseDetection(std::vector<RigidBodyBox>& rigidBodies);
	void narrowPhaseDetection(const std::vector<std::pair<Primitive*, Primitive*>>& potentialCollisions);

    
};
