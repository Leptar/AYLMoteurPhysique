#pragma once
#include <vector>
#include "CollisionData.h"
#include "../World/WorldObject/Primitive.h"
#include "../World/WorldObject/Box.h"
#include "../World/WorldObject/Plane.h"
#include "../CorpsRigide/CorpsRigide.h"

// Note : Si CollisionType est utilisé dans la struct Contact (dans CollisionData.h), 
// il devrait idéalement être défini là-bas. Sinon, on le garde ici.
enum class CollisionType {
    Contact, Resting, Cable, Rod
};

class SystemeCollisionDetection
{
public:
    std::vector<Contact> detectedCollisions;

    SystemeCollisionDetection() = default;
    ~SystemeCollisionDetection() = default;

    void add(Primitive* p1, Primitive* p2, const Vector3D& point, const Vector3D& normal, float penetration, float restitution, CollisionType type);

    void addPlane(Primitive* p1, Primitive* p2, const Vector3D& point, const Vector3D& normal, float penetration, float restitution, CollisionType type);
    
    void remove(const Contact& contact);
    
    bool empty() const;
    void clear();
    
    void resolveAll(); 
    
    std::size_t count() const;
        
    void addRodConstraint(Primitive* p1, Primitive* p2, float length);
    void addCableConstraint(Primitive* p1, Primitive* p2, float maxLength, float restitution);

    void DetectBoxPlane(Box* box, Plane* plane);
};