#pragma once
#include "AABB.h"
#include "ofEvent.h"
#include "WorldObject/Primitive.h"

#include <array>
#include <memory>

class Octree
{
public:
    Octree(const AABB& NewArea);

    void subdivide();

    bool insert(Primitive* object,const AABB& NewArea);

    std::vector<Primitive*> request(const AABB& otherBounds);

    void collectBounds(std::vector<AABB>& outNodes) const;

    bool containsEntiraly(const AABB& other) const;
private:
    // Position du sommet AABB min(minx, miny, minz) et max(maxx, maxy, maxz)
    AABB Area;

    int MAX_SUBDIVIDE = 8;

    std::vector<Primitive*> Objets;

	// représentation 3d
	std::array<std::unique_ptr<Octree>, 8> children;

    bool bHasBeenSubdivide = false;
};
