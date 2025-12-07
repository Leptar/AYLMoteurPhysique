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
            // A recule (inverse de la normale)
            Vector3D posChange = movePerIMass.scalar(invMassA).scalar(-1.0f); 
            bodyA->setPosition(bodyA->getPosition() + posChange);
        }
        if (bodyB) {
            // B avance (sens de la normale)
            Vector3D posChange = movePerIMass.scalar(invMassB);
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
            Vector3D angularChange = iITWorldA * impulsiveTorque;
            // On inverse le signe car impulse est négatif pour A (3ème loi de Newton)
            bodyA->setVelociteAngulaire(bodyA->getVelociteAngulaire() + angularChange.scalar(-1.0f));
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
    if (!box || !plane || !box->corpsRigide) return;

    // La normale et l'offset du plan sont déjà en coordonnées monde.
    const Vector3D& planeNormal = plane->normal;
    const float planeOffset = plane->PlaneOffset;

    // Obtenir la matrice de transformation pour passer les sommets de la boîte en coordonnées monde.
    Matrix4 boxToWorld = box->GetTransformMatrix();

    // Définir les 8 sommets de la boîte dans son espace local.
    float hx = box->HalfExtent.x;
    float hy = box->HalfExtent.y;
    float hz = box->HalfExtent.z;

    std::vector<Vector3D> verticesLocal = {
        { hx,  hy,  hz}, {-hx,  hy,  hz},
        { hx, -hy,  hz}, {-hx, -hy,  hz},
        { hx,  hy, -hz}, {-hx,  hy, -hz},
        { hx, -hy, -hz}, {-hx, -hy, -hz}
    };

    float minDistance = std::numeric_limits<float>::max();
    Vector3D deepestVertex;

    // Trouver le sommet le plus pénétrant
    for (const auto& vertexLocal : verticesLocal)
    {
        Vector3D vertexWorld = boxToWorld * vertexLocal;
        float distance = vertexWorld.dot(planeNormal) + planeOffset;

        if (distance < minDistance)
        {
            minDistance = distance;
            deepestVertex = vertexWorld;
        }
    }

    // S'il y a pénétration (la plus petite distance est négative), on génère UN SEUL contact.
    if (minDistance <= 0.0f)
    {
        // Le point de contact est le sommet le plus profond.
        // La pénétration est la valeur absolue de la distance.
        add(box, plane, deepestVertex, planeNormal, -minDistance, 0.5f, collision_type::Contact);
    }
}

void SystemeCollisionDetection::DetectBoxBox(Box* boxA, Box* boxB)
{
    if (!boxA || !boxB || !boxA->corpsRigide || !boxB->corpsRigide)
    {
        return;
    }

    const Vector3D centerA = boxA->corpsRigide->getPosition();
    const Vector3D centerB = boxB->corpsRigide->getPosition();

    const float radiusA = boxA->HalfExtent.GetNorm();
    const float radiusB = boxB->HalfExtent.GetNorm();

    Vector3D diff = centerB - centerA;
    const float distance = diff.GetNorm();
    const float combinedRadius = radiusA + radiusB;

    if (distance >= combinedRadius || distance <= 1e-4f)
    {
        return;
    }

    Vector3D normal = diff.scalar(1.f / distance);
    const float penetration = combinedRadius - distance;

    const Vector3D contactPoint = centerA + normal.scalar(radiusA - penetration * 0.5f);

    add(boxA, boxB, contactPoint, normal, penetration, 0.5f, collision_type::Contact);
}