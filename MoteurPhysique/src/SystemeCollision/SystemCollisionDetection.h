#pragma once
#include <vector>
#include "particule.h"

enum class CollisionType {
    Contact, Resting, Cable, Rod
};

struct Collision {
    Particule* p1;
    Particule* p2; // nullptr si c’est un plan/point fixe
    Vector3D contactNormal; // direction de la réaction (du point de vue p1)
    float penetration; // profondeur d’interpénétration
    float restitution; // coefficient de rebond (0 = inélastique, 1 = élastique)
    CollisionType type;
    
    bool equal(const Particule* comp_p1, const Particule* comp_p2) const {
        return p1 == comp_p1 && p2 == comp_p2;
    }
    
};

class SystemCollisionDetection
{
public:
    std::vector<Collision> detectedCollisions;

    void add(Particule* p1, Particule* p2, float restitution, CollisionType type);
    void remove(const Collision& collision);
    bool clear();
    void resolveAll();
    static bool IsColliding(Particule* p1, Particule* p2);
    void addRodConstraint(Particule* p1, Particule* p2, float length);
    void addCableConstraint(Particule* p1, Particule* p2, float maxLength, float restitution);
};
