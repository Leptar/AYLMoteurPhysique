#include "Octree.h"

#include <algorithm>

Octree::Octree(const AABB& newBounds, int maxObjectsPerNode) : Area(newBounds)
{
    MAX_SUBDIVIDE = std::max(1, maxObjectsPerNode);
}

void Octree::subdivide()
{
    Vector3D center = Area.getCenter();
    Vector3D min = Area.min;
    Vector3D max = Area.max;

    // zyx
    children[0] = std::make_unique<Octree>(
        AABB(min, center),
        MAX_SUBDIVIDE
        );
    children[1] = std::make_unique<Octree>(
        AABB(
            Vector3D(center.x, min.y,    min.z),
            Vector3D(max.x,    center.y, center.z)
            )
        ,
        MAX_SUBDIVIDE
        );
    children[2] = std::make_unique<Octree>(
        AABB(
            Vector3D(min.x,    center.y, min.z),
            Vector3D(center.x, max.y,    center.z)
            )
        ,
        MAX_SUBDIVIDE
        );
    children[3] = std::make_unique<Octree>(
        AABB(
            Vector3D(center.x, center.y, min.z),
            Vector3D(max.x,    max.y,    center.z)
            )
        ,
        MAX_SUBDIVIDE
        );
    children[4] = std::make_unique<Octree>(
        AABB(
            Vector3D(min.x,    min.y,    center.z),
            Vector3D(center.x, center.y, max.z)
            )
        ,
        MAX_SUBDIVIDE
        );
    children[5] = std::make_unique<Octree>(
        AABB(
            Vector3D(center.x, min.y,    center.z),
            Vector3D(max.x,    center.y, max.z)
            )
        ,
        MAX_SUBDIVIDE
        );
    children[6] = std::make_unique<Octree>(
        AABB(
            Vector3D(min.x,    center.y, center.z),
            Vector3D(center.x, max.y,    max.z)
            )
        ,
        MAX_SUBDIVIDE
        );
    children[7] = std::make_unique<Octree>(
        AABB(center, max),
        MAX_SUBDIVIDE
        );

    bHasBeenSubdivide = true;
}

bool Octree::insert(Primitive* object, const AABB& objectBounds)
{
    // Si l'objet n'est pas dans ce noeud, on arrête.
    if (!Area.intersects(objectBounds))
    {
        return false;
    }

    // Si le noeud est déjà subdivise, on essaie de pousser l'objet vers un enfant.
    if (bHasBeenSubdivide)
    {
        // Tente d'inserer dans un enfant si l'objet est entièrement contenu dedans.
        if (children[0]->containsEntiraly(objectBounds)) return children[0]->insert(object, objectBounds);
        if (children[1]->containsEntiraly(objectBounds)) return children[1]->insert(object, objectBounds);
        if (children[2]->containsEntiraly(objectBounds)) return children[2]->insert(object, objectBounds);
        if (children[3]->containsEntiraly(objectBounds)) return children[3]->insert(object, objectBounds);
        if (children[4]->containsEntiraly(objectBounds)) return children[4]->insert(object, objectBounds);
        if (children[5]->containsEntiraly(objectBounds)) return children[5]->insert(object, objectBounds);
        if (children[6]->containsEntiraly(objectBounds)) return children[6]->insert(object, objectBounds);
        if (children[7]->containsEntiraly(objectBounds)) return children[7]->insert(object, objectBounds);
    }

    // Si on arrive ici, soit le noeud est une feuille, soit l'objet chevauche les frontieres des enfants.
    // On ajoute donc l'objet à ce noeud.
    Objets.emplace_back(object);

    // Si le noeud est une feuille, qu'il est plein et qu'on peut encore le diviser.
    if (!bHasBeenSubdivide && Objets.size() > MAX_SUBDIVIDE)
    {
        subdivide();

        // On essaie de redistribuer les objets de ce noeud vers les enfants.
        std::vector<Primitive*> RemainingObjects;
        for (Primitive* obj : Objets)
        {
            bool bPushedToChild = false;
            const AABB& objBounds = obj->corpsRigide->worldAABB;

            if (children[0]->containsEntiraly(objBounds)) { children[0]->insert(obj, objBounds); bPushedToChild = true; }
            else if (children[1]->containsEntiraly(objBounds)) { children[1]->insert(obj, objBounds); bPushedToChild = true; }
            else if (children[2]->containsEntiraly(objBounds)) { children[2]->insert(obj, objBounds); bPushedToChild = true; }
            else if (children[3]->containsEntiraly(objBounds)) { children[3]->insert(obj, objBounds); bPushedToChild = true; }
            else if (children[4]->containsEntiraly(objBounds)) { children[4]->insert(obj, objBounds); bPushedToChild = true; }
            else if (children[5]->containsEntiraly(objBounds)) { children[5]->insert(obj, objBounds); bPushedToChild = true; }
            else if (children[6]->containsEntiraly(objBounds)) { children[6]->insert(obj, objBounds); bPushedToChild = true; }
            else if (children[7]->containsEntiraly(objBounds)) { children[7]->insert(obj, objBounds); bPushedToChild = true; }

            // Si l'objet n'a pas pu être pousse, il reste dans le noeud parent.
            if (!bPushedToChild)
            {
                RemainingObjects.push_back(obj);
            }
        }
        Objets = RemainingObjects;
    }
    return true;
}

std::vector<Primitive*> Octree::request(const AABB& otherBounds)
{
    std::vector<Primitive*> FindObjects;

    // Si la zone de recherche n'intersecte pas ce noeud, on retourne une liste vide.
    if (!Area.intersects(otherBounds)) { return FindObjects; }

    // Ajoute les objets de ce noeud qui intersectent la zone de recherche.
    for (auto Obj : Objets)
    {
        if (otherBounds.intersects(Obj->corpsRigide->worldAABB))
        {
            FindObjects.push_back(Obj);
        }
    }

    // Si le noeud n'est pas subdivise, on a fini.
    if (!bHasBeenSubdivide) { return FindObjects; }

    // Sinon, on ajoute les resultats des enfants.
    auto childResult = children[0]->request(otherBounds);
    FindObjects.insert(FindObjects.end(), childResult.begin(), childResult.end());
    childResult = children[1]->request(otherBounds);
    FindObjects.insert(FindObjects.end(), childResult.begin(), childResult.end());
    childResult = children[2]->request(otherBounds);
    FindObjects.insert(FindObjects.end(), childResult.begin(), childResult.end());
    childResult = children[3]->request(otherBounds);
    FindObjects.insert(FindObjects.end(), childResult.begin(), childResult.end());
    childResult = children[4]->request(otherBounds);
    FindObjects.insert(FindObjects.end(), childResult.begin(), childResult.end());
    childResult = children[5]->request(otherBounds);
    FindObjects.insert(FindObjects.end(), childResult.begin(), childResult.end());
    childResult = children[6]->request(otherBounds);
    FindObjects.insert(FindObjects.end(), childResult.begin(), childResult.end());
    childResult = children[7]->request(otherBounds);
    FindObjects.insert(FindObjects.end(), childResult.begin(), childResult.end());

    return FindObjects;
}

bool Octree::containsEntiraly(const AABB& other) const
{
    return Area.contains(other);
}
