#pragma once
#include <vector>

#include "CorpsRigide.h"

/*
 * Classe pour le traitement des collisions de la phase 4
 */

struct Contact
{
    CorpsRigide* c1;
    CorpsRigide* c2;
    Vector3D contactNormal; // direction de la réaction (du point de vue p1)
    float penetration; // profondeur d’interpénétration
    float restitution; // coefficient de rebond (0 = inélastique, 1 = élastique)
    float friction;
    Vector3D contactPoint;

    bool equal(const CorpsRigide* comp_c1, const CorpsRigide* comp_c2) const
    {
        return c1 == comp_c1 && c2 == comp_c2;
    }
};

class CollisionData
{
public:
    std::vector<Contact> contacts;
    int RemainingContacts;

    void add(Contact contact);
    void remove(const Contact& contact);
    void clear();
    
};
