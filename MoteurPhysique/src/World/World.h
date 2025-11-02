#pragma once

#include <vector>
#include "../particule.h"
#include "../ForceGenerator/ParticuleForceRegistry.h"
#include "../SystemeCollision/SystemCollisionDetection.h"

class World
{
public:
    World();
    ~World();

    /**
     * @brief
     * Appelle tous les systèmes dans l'ordre (Forces, Intégration, Collisions) pour l'exécuter à chaque frame.
     */
    void update(float deltaTime);

    /**
     * @brief Ajoute une particule à la simulation.
     */
    void addParticule(Particule* particule);

    /**
     * @brief Donne accès au registre des forces pour y lier des générateurs.
     */
    ParticuleForceRegistry* getForceRegistry();

    /**
     * @brief Donne accès au système de détection pour y ajouter
     * des contraintes (tiges, câbles, plans).
     */
    SystemCollisionDetection* getCollisionDetector();

private:
    std::vector<Particule*> m_particules;

    ParticuleForceRegistry m_forceRegistry;
    SystemCollisionDetection m_collisionDetector;
};