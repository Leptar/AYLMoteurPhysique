#include "SystemeCollisionDetection.h"
#include <algorithm>

void SystemeCollisionDetection::add(Primitive* p1, Primitive* p2, const Vector3D& point, const Vector3D& normal, float penetration, float restitution, collision_type type)
{
    if (!p1 || !p2) return;

    Contact contact;
    // Conversion : On récupère les CorpsRigides associés aux primitives via le membre 'corpsRigide'
    contact.c1 = p1->corpsRigide; 
    contact.c2 = p2->corpsRigide;
    
    contact.contactPoint = point;
    contact.contactNormal = normal;
    contact.penetration = penetration;
    contact.restitution = restitution;
    
    // La structure Contact a un champ friction, mais la méthode add ne le passait pas.
    // On met une valeur par défaut raisonnable.
    contact.friction = 0.5f; 

    // Note : Si tu veux stocker le CollisionType, il faudra l'ajouter dans la struct Contact.
    contact.type = type; 

    detectedCollisions.push_back(contact);
}

void SystemeCollisionDetection::addPlane(Primitive* p1, Primitive* p2, const Vector3D& point, const Vector3D& normal, float penetration, float restitution, collision_type type)
{
    if (!p1 || !p2) return;

    Contact contact;
    contact.c1 = p1->corpsRigide;
    contact.c2 = p2->corpsRigide;
    
    contact.contactPoint = point;
    contact.contactNormal = normal;
    contact.penetration = penetration;
    contact.restitution = restitution;
    contact.friction = 0.5f; // Valeur par défaut

    contact.type = type;

    detectedCollisions.push_back(contact);
}

void SystemeCollisionDetection::remove(const Contact& contact)
{
    // On retire la collision qui correspond aux corps rigides donnés
    detectedCollisions.erase(
        std::remove_if(detectedCollisions.begin(), detectedCollisions.end(),
            [&](const Contact& c) { 
                // On utilise la méthode equal de la struct Contact qui compare c1 et c2
                return c.equal(contact.c1, contact.c2); 
            }),
        detectedCollisions.end());
}

bool SystemeCollisionDetection::empty() const
{
    return detectedCollisions.empty();
}

void SystemeCollisionDetection::clear()
{
    detectedCollisions.clear();
}

void SystemeCollisionDetection::resolveAll()
{
    if (detectedCollisions.empty()) return;

    // 1. Résolution de l'Interpénétration (Correction de Position)
    // On sépare les objets pour qu'ils ne soient plus l'un dans l'autre
    for (auto& contact : detectedCollisions)
    {
        CorpsRigide* bodyA = contact.c1;
        CorpsRigide* bodyB = contact.c2;

        // On ignore si aucun corps n'est présent
        if (!bodyA && !bodyB) continue;
        if (contact.penetration <= 0) continue;

        float invMassA = bodyA ? bodyA->getInverseMasse() : 0.0f;
        float invMassB = bodyB ? bodyB->getInverseMasse() : 0.0f;
        float totalInvMass = invMassA + invMassB;

        // Si masse infinie pour les deux (ex: deux objets statiques), pas de déplacement
        if (totalInvMass <= 0) continue; 

        // La correction est proportionnelle à la masse inverse (l'objet léger bouge plus)
        Vector3D movePerIMass = contact.contactNormal.scalar(contact.penetration / totalInvMass);

        if (bodyA) {
            // A est déplacé dans la direction de la normale pour sortir de B
            Vector3D posChange = movePerIMass.scalar(invMassA);
            bodyA->setPosition(bodyA->getPosition() + posChange);
        }
        if (bodyB) {
            // B recule (direction opposée à la normale)
            Vector3D posChange = movePerIMass.scalar(invMassB).scalar(-1.0f);
            bodyB->setPosition(bodyB->getPosition() + posChange);
        }
    }

    // 2. Résolution des Vélocités (Impulsions)
    // On applique des forces instantanées pour faire rebondir les objets
    for (auto& contact : detectedCollisions)
    {
        CorpsRigide* bodyA = contact.c1;
        CorpsRigide* bodyB = contact.c2;

        float invMassA = bodyA ? bodyA->getInverseMasse() : 0.0f;
        float invMassB = bodyB ? bodyB->getInverseMasse() : 0.0f;
        float totalInvMass = invMassA + invMassB;

        if (totalInvMass <= 0) continue;

        // --- Calcul des propriétés relatives ---
        
        // Vecteurs position relative (Centre de masse -> Point de contact)
        Vector3D rA = bodyA ? (contact.contactPoint - bodyA->getPosition()) : Vector3D(0,0,0);
        Vector3D rB = bodyB ? (contact.contactPoint - bodyB->getPosition()) : Vector3D(0,0,0);

        // Tenseurs d'inertie inverses en monde (Matrix3)
        // Utilisation de la méthode spécifique de ta classe CorpsRigide
        Matrix3 iITWorldA = bodyA ? bodyA->getInverseInertiaTensorWorld() : Matrix3();
        Matrix3 iITWorldB = bodyB ? bodyB->getInverseInertiaTensorWorld() : Matrix3();

        // --- Calcul de la vitesse de fermeture ---

        // Vitesse au point de contact = V_lineaire + (V_angulaire x R)
        Vector3D velA = bodyA ? (bodyA->getVelocite() + bodyA->getVelociteAngulaire().cross(rA)) : Vector3D(0,0,0);
        Vector3D velB = bodyB ? (bodyB->getVelocite() + bodyB->getVelociteAngulaire().cross(rB)) : Vector3D(0,0,0);

        // Vitesse relative (B - A)
        Vector3D relativeVelocity = velB - velA;

        // Vitesse le long de la normale de contact
        float separatingVelocity = relativeVelocity.dot(contact.contactNormal);

        // Si les objets s'éloignent déjà, on ne fait rien
        if (separatingVelocity > 0)
        {
            continue;
        }

        // --- Calcul de l'Impulsion scalaire (j) ---

        // Nouvelle vitesse de séparation désirée
        // j = -(1 + e) * v_rel
        float newSepVelocity = -separatingVelocity * (1.0f + contact.restitution);

        // Check pour éviter la micro-vibration au repos (resting contact)
        // Si la vitesse est très faible, on annule le rebond
        // On approxime la vitesse induite par l'accélération (gravité) sur une frame
        // Pour l'instant, seuil simple :
        if (std::abs(separatingVelocity) < 0.2f) 
        {
            newSepVelocity = 0.0f; 
        }

        float deltaVelocity = newSepVelocity - separatingVelocity;

        // Dénominateur de l'impulsion (Masse linéaire + Masse angulaire)
        // angularComponent = (r x n) . (I^-1 * (r x n))
        float angularEffectA = 0;
        float angularEffectB = 0;

        if (bodyA) {
            Vector3D torquePerUnitImpulse = rA.cross(contact.contactNormal);
            Vector3D rotationPerUnitImpulse = iITWorldA * torquePerUnitImpulse;
            angularEffectA = rotationPerUnitImpulse.dot(torquePerUnitImpulse);
        }

        if (bodyB) {
            Vector3D torquePerUnitImpulse = rB.cross(contact.contactNormal);
            Vector3D rotationPerUnitImpulse = iITWorldB * torquePerUnitImpulse;
            angularEffectB = rotationPerUnitImpulse.dot(torquePerUnitImpulse);
        }

        float totalInverseMass = totalInvMass + angularEffectA + angularEffectB;

        // Impulsion finale j
        float impulseScalar = deltaVelocity / totalInverseMass;
        
        Vector3D impulse = contact.contactNormal.scalar(impulseScalar);

        // --- Application de l'Impulsion ---

        if (bodyA)
        {
            // Vitesse Linéaire : v_new = v_old - (P * invMass)
            Vector3D deltaV = impulse.scalar(-invMassA);
            bodyA->setVelocite(bodyA->getVelocite() + deltaV);
            
            // Vitesse Angulaire : w_new = w_old - (I^-1 * (r x P))
            Vector3D impulsiveTorque = rA.cross(impulse);
            Vector3D angularChange = (iITWorldA * impulsiveTorque).scalar(-1.0f);
            bodyA->setVelociteAngulaire(bodyA->getVelociteAngulaire() + angularChange);
        }

        if (bodyB)
        {
            // Vitesse Linéaire : v_new = v_old + (P * invMass)
            Vector3D deltaV = impulse.scalar(invMassB);
            bodyB->setVelocite(bodyB->getVelocite() + deltaV);

            // Vitesse Angulaire : w_new = w_old + (I^-1 * (r x P))
            Vector3D impulsiveTorque = rB.cross(impulse);
            Vector3D angularChange = iITWorldB * impulsiveTorque;
            bodyB->setVelociteAngulaire(bodyB->getVelociteAngulaire() + angularChange);
        }
    }
}

std::size_t SystemeCollisionDetection::count() const
{
    return detectedCollisions.size();
}

void SystemeCollisionDetection::addRodConstraint(Primitive* p1, Primitive* p2, float length)
{
    if (!p1 || !p2) return;

    Contact contact;
    contact.c1 = p1->corpsRigide;
    contact.c2 = p2->corpsRigide;
    
    // Note : Sans le champ 'type' dans Contact, le résolveur ne saura pas que c'est une tige.
    contact.type = collision_type::Rod;
    
    contact.friction = 0.0f;
    contact.restitution = 0.0f;

    detectedCollisions.push_back(contact);
}

void SystemeCollisionDetection::addCableConstraint(Primitive* p1, Primitive* p2, float maxLength, float restitution)
{
    if (!p1 || !p2) return;

    Contact contact;
    contact.c1 = p1->corpsRigide;
    contact.c2 = p2->corpsRigide;
    
    contact.restitution = restitution;
    contact.friction = 0.0f;
    
    contact.type = collision_type::Cable;

    detectedCollisions.push_back(contact);
}

// -----------------------------------------------------------------------
// IMPLÉMENTATION BOÎTE - PLAN qui va généraliser presque tous les contacts possibles
// -----------------------------------------------------------------------
void SystemeCollisionDetection::DetectBoxPlane(Box* box, Plane* plane)
{
    // Si l'un des objets n'existe pas, on ne fait rien.
    if (!box || !plane || !box->corpsRigide) return;

    // 1. La normale et la distance du plan sont déjà en coordonnées monde.
    const Vector3D& planeNormalWorld = plane->normal;
    float planeOffset = plane->PlaneOffset;

    // 2. On récupère la matrice de transformation de la boîte pour passer ses sommets en monde.
    Matrix4 boxToWorld = box->GetTransformMatrix();

    // 3. Les 8 sommets de la boîte en coordonnées locales
    float hx = box->HalfExtent.x;
    float hy = box->HalfExtent.y;
    float hz = box->HalfExtent.z;

    std::vector<Vector3D> verticesLocal = {
        Vector3D( hx,  hy,  hz), Vector3D(-hx,  hy,  hz),
        Vector3D( hx, -hy,  hz), Vector3D(-hx, -hy,  hz),
        Vector3D( hx,  hy, -hz), Vector3D(-hx,  hy, -hz),
        Vector3D( hx, -hy, -hz), Vector3D(-hx, -hy, -hz)
    };

    // 4. Test d'intersection : trouver le sommet le plus pénétrant
    float maxPenetration = -std::numeric_limits<float>::max();
    Vector3D deepestVertexWorld;

    for (const auto& vertexLocal : verticesLocal) {
        // Q: Sommet en World
        Vector3D vertexWorld = boxToWorld * vertexLocal;

        // Distance signée du sommet au plan: d = (n . Q) - offset
        float distance = planeNormalWorld.dot(vertexWorld) - planeOffset;

        // Si la distance est négative, il y a pénétration.
        // On cherche la plus grande pénétration (la distance la plus négative).
        if (distance < 0 && distance > maxPenetration) { // distance est négatif
            maxPenetration = distance;
            deepestVertexWorld = vertexWorld;
        }
    }

    // 5. Si on a trouvé un sommet pénétrant, on génère UN SEUL contact.
    if (maxPenetration != -std::numeric_limits<float>::max())
    {
        // La pénétration est la valeur absolue de la distance.
        float penetration = -maxPenetration;

        // Le point de contact est le sommet projeté sur le plan.
        Vector3D contactPoint = deepestVertexWorld - planeNormalWorld.scalar(maxPenetration);

        // Ajout du contact via la méthode add.
        // Le plan n'a pas de corps rigide (nullptr), ce qui est géré par la résolution.
        add(box, plane, contactPoint, planeNormalWorld, penetration, 0.5f, collision_type::Contact);
    }
}

// -----------------------------------------------------------------------
// IMPLÉMENTATION BOÎTE - BOÎTE (SAT)
// -----------------------------------------------------------------------

namespace {
    // Calcule la projection d'une boîte sur un axe et retourne l'intervalle [min, max]
    void projectBoxOntoAxis(const Box* box, const Vector3D& axis, float& min, float& max)
    {
        Matrix4 transform = box->GetTransformMatrix();
        Vector3D center = transform * Vector3D(0, 0, 0);

        // Les axes de la boîte en coordonnées monde
        Vector3D axisX = (transform * Vector3D(1, 0, 0) - center).normalize();
        Vector3D axisY = (transform * Vector3D(0, 1, 0) - center).normalize();
        Vector3D axisZ = (transform * Vector3D(0, 0, 1) - center).normalize();

        // Rayon de la boîte projeté sur l'axe
        float radius =
            box->HalfExtent.x * std::abs(axis.dot(axisX)) +
            box->HalfExtent.y * std::abs(axis.dot(axisY)) +
            box->HalfExtent.z * std::abs(axis.dot(axisZ));

        float centerProjection = center.dot(axis);
        min = centerProjection - radius;
        max = centerProjection + radius;
    }

    // Teste si deux intervalles se chevauchent et retourne la profondeur de pénétration
    float getOverlap(float minA, float maxA, float minB, float maxB)
    {
        return std::max(0.0f, std::min(maxA, maxB) - std::max(minA, minB));
    }
}

void SystemeCollisionDetection::DetectBoxBox(Box* box1, Box* box2)
{
    if (!box1 || !box2 || !box1->corpsRigide || !box2->corpsRigide) return;

    Matrix4 transform1 = box1->GetTransformMatrix();
    Matrix4 transform2 = box2->GetTransformMatrix();

    Vector3D center1 = transform1 * Vector3D(0, 0, 0);
    Vector3D center2 = transform2 * Vector3D(0, 0, 0);

    // Axes locaux des boîtes en coordonnées monde
    Vector3D axes1[3] = {
        (transform1 * Vector3D(1, 0, 0) - center1).normalize(),
        (transform1 * Vector3D(0, 1, 0) - center1).normalize(),
        (transform1 * Vector3D(0, 0, 1) - center1).normalize()
    };
    Vector3D axes2[3] = {
        (transform2 * Vector3D(1, 0, 0) - center2).normalize(),
        (transform2 * Vector3D(0, 1, 0) - center2).normalize(),
        (transform2 * Vector3D(0, 0, 1) - center2).normalize()
    };

    // Stocker tous les axes à tester
    std::vector<Vector3D> testAxes;
    for (int i = 0; i < 3; ++i) testAxes.push_back(axes1[i]);
    for (int i = 0; i < 3; ++i) testAxes.push_back(axes2[i]);
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            Vector3D cross = axes1[i].cross(axes2[j]);
            if (cross.GetSquareNorm() > 1e-6) { // Éviter les axes nuls ou quasi-nuls
                testAxes.push_back(cross.normalize());
            }
        }
    }

    float minOverlap = std::numeric_limits<float>::max();
    Vector3D collisionNormal;

    // Parcourir tous les axes potentiels
    for (const auto& axis : testAxes) {
        float min1, max1, min2, max2;
        projectBoxOntoAxis(box1, axis, min1, max1);
        projectBoxOntoAxis(box2, axis, min2, max2);

        // Si les projections ne se chevauchent pas, il n'y a pas de collision
        if (max1 < min2 || max2 < min1) {
            return; // Axe séparateur trouvé, pas de collision.
        }

        // Calculer le chevauchement et garder le minimum
        float overlap = getOverlap(min1, max1, min2, max2);
        if (overlap < minOverlap) {
            minOverlap = overlap;
            collisionNormal = axis;
        }
    }

    // Si on arrive ici, les boîtes sont en collision.
    // `minOverlap` est la pénétration et `collisionNormal` est la normale de collision.

    // S'assurer que la normale pointe de box1 vers box2
    Vector3D direction = center2 - center1;
    if (collisionNormal.dot(direction) < 0) {
        collisionNormal = collisionNormal.scalar(-1.0f);
    }

    // --- Calcul du point de contact (Amélioré) ---
    // On trouve le sommet de box2 le plus profond dans la direction de la normale de collision.
    // C'est une bonne approximation pour les collisions sommet-face.
    Vector3D contactPoint;
    float maxSeparation = -std::numeric_limits<float>::max();

    float hx2 = box2->HalfExtent.x;
    float hy2 = box2->HalfExtent.y;
    float hz2 = box2->HalfExtent.z;
    std::vector<Vector3D> verticesLocal2 = {
        Vector3D( hx2,  hy2,  hz2), Vector3D(-hx2,  hy2,  hz2),
        Vector3D( hx2, -hy2,  hz2), Vector3D(-hx2, -hy2,  hz2),
        Vector3D( hx2,  hy2, -hz2), Vector3D(-hx2,  hy2, -hz2),
        Vector3D( hx2, -hy2, -hz2), Vector3D(-hx2, -hy2, -hz2)
    };

    for(const auto& vertexLocal : verticesLocal2)
    {
        Vector3D vertexWorld = transform2 * vertexLocal;
        float separation = (vertexWorld - center1).dot(collisionNormal);

        if (separation > maxSeparation)
        {
            maxSeparation = separation;
            contactPoint = vertexWorld;
        }
    }

    add(box1, box2, contactPoint, collisionNormal, minOverlap, 0.5f, collision_type::Contact);
}