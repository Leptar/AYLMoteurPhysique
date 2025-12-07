#include "Octree.h"

#include "of3dGraphics.h"

Octree::Octree(const AABB& newBounds) : Area(newBounds)
{
}

void Octree::subdivide()
{
    Vector3D center = Area.getCenter();
    Vector3D min = Area.min;
    Vector3D max = Area.max;

    // zyx
    children[0] = std::make_unique<Octree>(
        AABB(min, center)
    );
    children[1] = std::make_unique<Octree>(
        AABB(
            Vector3D(center.x, min.y, min.z),
            Vector3D(max.x, center.y, center.z)
        )
    );
    children[2] = std::make_unique<Octree>(
        AABB(
            Vector3D(min.x, center.y, min.z),
            Vector3D(center.x, max.y, center.z)
        )
    );
    children[3] = std::make_unique<Octree>(
        AABB(
            Vector3D(center.x, center.y, min.z),
            Vector3D(max.x, max.y, center.z)
        )
    );
    children[4] = std::make_unique<Octree>(
        AABB(
            Vector3D(min.x, min.y, center.z),
            Vector3D(center.x, center.y, max.z)
        )
    );
    children[5] = std::make_unique<Octree>(
        AABB(
            Vector3D(center.x, min.y, center.z),
            Vector3D(max.x, center.y, max.z)
        )
    );
    children[6] = std::make_unique<Octree>(
        AABB(
            Vector3D(min.x, center.y, center.z),
            Vector3D(center.x, max.y, max.z)
        )
    );
    children[7] = std::make_unique<Octree>(
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
        for (auto& child : children)
        {
            if (child->containsEntiraly(objectBounds))
            {
                return child->insert(object, objectBounds);
            }
        }
    }

    // Si on arrive ici, soit le noeud est une feuille, soit l'objet chevauche les frontieres des enfants.
    // On ajoute donc l'objet à ce noeud.
    Objets.emplace_back(object);

    // Si le noeud est une feuille, qu'il est plein et qu'on peut encore le diviser.
    if (!bHasBeenSubdivide && Objets.size() > MAX_SUBDIVIDE)
    {
        subdivide();

        std::vector<Primitive*> remainingObjects;
        remainingObjects.reserve(Objets.size());

        // On redistribue les objets qui peuvent maintenant aller dans les enfants.
        for (Primitive* obj : Objets) {
            bool movedToChild = false;
            const AABB& objBounds = obj->corpsRigide->worldAABB;
            for (auto& child : children) {
                if (child->containsEntiraly(objBounds)) {
                    child->insert(obj, objBounds);
                    movedToChild = true;
                    break; // L'objet a été déplacé, on passe au suivant.
                }
            }

            // Si l'objet n'a pas pu être déplacé, il reste dans ce noeud.
            if (!movedToChild) {
                remainingObjects.push_back(obj);
            }
        }

        // Met à jour la liste des objets du noeud courant.
        Objets = remainingObjects;
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

void Octree::generatePairsForNode(std::vector<std::pair<Primitive*, Primitive*>>& potentialCollisions)
{
    // Génère des paires pour les objets à l'intérieur de ce noeud.
    for (size_t i = 0; i < Objets.size(); ++i)
    {
        for (size_t j = i + 1; j < Objets.size(); ++j)
        {
            Primitive* p1 = Objets[i];
            Primitive* p2 = Objets[j];
            // Assure un ordre constant pour éviter les doublons (A,B) vs (B,A)
            if (p1 < p2)
            {
                potentialCollisions.emplace_back(p1, p2);
            }
            else
            {
                potentialCollisions.emplace_back(p2, p1);
            }
        }
    }
}

void Octree::generatePotentialCollisions(std::vector<std::pair<Primitive*, Primitive*>>& potentialCollisions)
{
    // 1. Générer les paires pour les objets contenus directement dans ce nœud.
    generatePairsForNode(potentialCollisions);

    // 2. Si le nœud est subdivisé, générer les paires entre les objets de ce nœud
    //    et les objets contenus directement dans les nœuds enfants.
    if (bHasBeenSubdivide)
    {
        // Pour chaque objet de ce nœud parent...
        for (Primitive* parentObject : Objets)
        {
            // ...on le teste contre tous les objets de tous les enfants.
            for (const auto& child : children)
            {
                for (Primitive* childObject : child->Objets)
                {
                    // On assure un ordre constant pour éviter les doublons.
                    if (parentObject < childObject) {
                        potentialCollisions.emplace_back(parentObject, childObject);
                    } else {
                        potentialCollisions.emplace_back(childObject, parentObject);
                    }
                }
            }
        }
        // 3. Appel récursif sur les enfants pour qu'ils fassent de même.
        for (auto& child : children) child->generatePotentialCollisions(potentialCollisions);
    }
}

void Octree::draw() const
{
    ofPushStyle();
    ofNoFill(); // On veut voir à travers les boîtes
    ofSetColor(ofColor::cyan);

    Vector3D center = Area.getCenter();
    Vector3D size = Area.getSize();

    // Dessine la boîte représentant ce noeud de l'Octree
    ofDrawBox(center.x, center.y, center.z, size.x, size.y, size.z);

    // Si le noeud est subdivisé, on appelle récursivement le dessin sur les enfants
    if (bHasBeenSubdivide)
    {
        for (const auto& child : children)
        {
            if (child) child->draw();
        }
    }
    ofPopStyle();
}
