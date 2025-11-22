#pragma once
#include "AABB.h"
#include "ofEvent.h"
#include "WorldObject/Primitive.h"

class Octree
{
public:
    Octree(const AABB& NewArea);

    void subdivide();
    
    bool insert(Primitive* object,const AABB& NewArea);
    
    std::vector<Primitive*> request(const AABB& otherBounds);

    bool containsEntiraly(const AABB& other) const;    
private:
    // Position du sommet AABB min(minx, miny, minz) et max(maxx, maxy, maxz)
    AABB Area;

    int MAX_SUBDIVIDE = 8;
    
    std::vector<Primitive*> Objets;

    // nommage example Octree_100 : z=1; y=0; x=0;
    // Representation 3D de la separation
    Octree* Octree_000 = nullptr;
    Octree* Octree_001 = nullptr;
    Octree* Octree_010 = nullptr;
    Octree* Octree_011 = nullptr;
    Octree* Octree_100 = nullptr;
    Octree* Octree_101 = nullptr;
    Octree* Octree_110 = nullptr;
    Octree* Octree_111 = nullptr;

    bool bHasBeenSubdivide = false;
};
