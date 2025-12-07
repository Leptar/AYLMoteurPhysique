#pragma once
#include "AABB.h"
#include "ofEvent.h"
#include "WorldObject/Primitive.h"
#include "ofGraphics.h"

class Octree
{
public:
    Octree(const AABB& NewArea);

    void subdivide();

    bool insert(Primitive* object,const AABB& NewArea);

    std::vector<Primitive*> request(const AABB& otherBounds);

    void generatePotentialCollisions(std::vector<std::pair<Primitive*, Primitive*>>& potentialCollisions);

    bool containsEntiraly(const AABB& other) const;

    void draw() const;
private:
    // Position du sommet AABB min(minx, miny, minz) et max(maxx, maxy, maxz)
    AABB Area;

    int MAX_SUBDIVIDE = 8;

    std::vector<Primitive*> Objets;

	// représentation 3d
	std::array<std::unique_ptr<Octree>, 8> children;

    bool bHasBeenSubdivide = false;

    void generatePairsForNode(std::vector<std::pair<Primitive*, Primitive*>>& potentialCollisions);
};
