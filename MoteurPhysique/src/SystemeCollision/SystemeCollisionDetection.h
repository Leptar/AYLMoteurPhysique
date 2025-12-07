#pragma once
#include <vector>
#include "CollisionData.h"
#include "SystemCollisionDetection.h"
#include "../World/WorldObject/Primitive.h"
#include "../World/WorldObject/Box.h"
#include "../World/WorldObject/Plane.h"
#include "../CorpsRigide/CorpsRigide.h"


class SystemeCollisionDetection
{
public:
    std::vector<Contact> detectedCollisions;

    SystemeCollisionDetection() = default;
    ~SystemeCollisionDetection() = default;

    void add(Primitive* p1, Primitive* p2, const Vector3D& point, const Vector3D& normal, float penetration, float restitution, collision_type type);

    void addPlane(Primitive* p1, Primitive* p2, const Vector3D& point, const Vector3D& normal, float penetration, float restitution, collision_type type);
    
    void remove(const Contact& contact);
    
    bool empty() const;
    void clear();
    
    void resolveAll(); 
    
    std::size_t count() const;
        
    void addRodConstraint(Primitive* p1, Primitive* p2, float length);
    void addCableConstraint(Primitive* p1, Primitive* p2, float maxLength, float restitution);

    void DetectBoxPlane(Box* box, Plane* plane);
    void DetectBoxBox(Box* box1, Box* box2);
};