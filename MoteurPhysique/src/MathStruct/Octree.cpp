#include "Octree.h"

Octree::Octree(const AABB& newBounds) : Area(newBounds)
{
}

void Octree::subdivide()
{
    Vector3D center = Area.getCenter();
    Vector3D min = Area.min;
    Vector3D max = Area.max;

    // zyx
    Octree_000 = new Octree(
        AABB(min, center)
        );
    Octree_001 = new Octree(
        AABB(
            Vector3D(center.x, min.y,    min.z),
            Vector3D(max.x,    center.y, center.z)
            )
        );
    Octree_010 = new Octree(
        AABB(
            Vector3D(min.x,    center.y, min.z),
            Vector3D(center.x, max.y,    center.z)
            )
        );
    Octree_011 = new Octree(
        AABB(
            Vector3D(center.x, center.y, min.z),
            Vector3D(max.x,    max.y,    center.z)
            )
        );
    Octree_100 = new Octree(
        AABB(
            Vector3D(min.x,    min.y,    center.z),
            Vector3D(center.x, center.y, max.z)
            )
        );
    Octree_101 = new Octree(
        AABB(
            Vector3D(center.x, min.y,    center.z),
            Vector3D(max.x,    center.y, max.z)
            )
        );
    Octree_110 = new Octree(
        AABB(
            Vector3D(min.x,    center.y, center.z),
            Vector3D(center.x, max.y,    max.z)
            )
        );
    Octree_111 = new Octree(
        AABB(center, max)
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
        if (Octree_000->containsEntiraly(objectBounds)) return Octree_000->insert(object, objectBounds);
        if (Octree_001->containsEntiraly(objectBounds)) return Octree_001->insert(object, objectBounds);
        if (Octree_010->containsEntiraly(objectBounds)) return Octree_010->insert(object, objectBounds);
        if (Octree_011->containsEntiraly(objectBounds)) return Octree_011->insert(object, objectBounds);
        if (Octree_100->containsEntiraly(objectBounds)) return Octree_100->insert(object, objectBounds);
        if (Octree_101->containsEntiraly(objectBounds)) return Octree_101->insert(object, objectBounds);
        if (Octree_110->containsEntiraly(objectBounds)) return Octree_110->insert(object, objectBounds);
        if (Octree_111->containsEntiraly(objectBounds)) return Octree_111->insert(object, objectBounds);
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

            if (Octree_000->containsEntiraly(objBounds)) { Octree_000->insert(obj, objBounds); bPushedToChild = true; }
            else if (Octree_001->containsEntiraly(objBounds)) { Octree_001->insert(obj, objBounds); bPushedToChild = true; }
            else if (Octree_010->containsEntiraly(objBounds)) { Octree_010->insert(obj, objBounds); bPushedToChild = true; }
            else if (Octree_011->containsEntiraly(objBounds)) { Octree_011->insert(obj, objBounds); bPushedToChild = true; }
            else if (Octree_100->containsEntiraly(objBounds)) { Octree_100->insert(obj, objBounds); bPushedToChild = true; }
            else if (Octree_101->containsEntiraly(objBounds)) { Octree_101->insert(obj, objBounds); bPushedToChild = true; }
            else if (Octree_110->containsEntiraly(objBounds)) { Octree_110->insert(obj, objBounds); bPushedToChild = true; }
            else if (Octree_111->containsEntiraly(objBounds)) { Octree_111->insert(obj, objBounds); bPushedToChild = true; }

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
    auto childResult = Octree_000->request(otherBounds);
    FindObjects.insert(FindObjects.end(), childResult.begin(), childResult.end());
    childResult = Octree_001->request(otherBounds);
    FindObjects.insert(FindObjects.end(), childResult.begin(), childResult.end());
    childResult = Octree_010->request(otherBounds);
    FindObjects.insert(FindObjects.end(), childResult.begin(), childResult.end());
    childResult = Octree_011->request(otherBounds);
    FindObjects.insert(FindObjects.end(), childResult.begin(), childResult.end());
    childResult = Octree_100->request(otherBounds);
    FindObjects.insert(FindObjects.end(), childResult.begin(), childResult.end());
    childResult = Octree_101->request(otherBounds);
    FindObjects.insert(FindObjects.end(), childResult.begin(), childResult.end());
    childResult = Octree_110->request(otherBounds);
    FindObjects.insert(FindObjects.end(), childResult.begin(), childResult.end());
    childResult = Octree_111->request(otherBounds);
    FindObjects.insert(FindObjects.end(), childResult.begin(), childResult.end());

    return FindObjects;
}

bool Octree::containsEntiraly(const AABB& other) const
{
    return Area.contains(other);
}
