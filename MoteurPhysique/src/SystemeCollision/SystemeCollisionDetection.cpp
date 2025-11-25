#include "SystemCollisionDetection.h"

// Inclusions pour accéder aux membres internes des classes utilisées
#include "../World/RigidBodyBox.h" 
#include "../MathStruct/Matrix4.h" 
#include "../MathStruct/3DVector.h"
#include <cmath>

void SystemCollisionDetection::genererContactsBoitePlan(
    RigidBodyBox* boite, 
    const Plan& plan, 
    CollisionData* data)
{
    // Sécurité : pointeurs valides
    if (!boite || !data) return;

    // 1. Dimensions (Demi-mesures) de la boîte
    Vector3D h = boite->halfExtents; 

    // 2. Transformation (Locale -> Monde)
    // On récupère la transformation du CorpsRigide contenu dans la boîte
    Matrix4 transform = boite->body.getTransform(); 

    // 3. Définition des 8 sommets LOCAUX
    Vector3D localSommets[8] = {
        Vector3D(-h.x, -h.y, -h.z),
        Vector3D(-h.x, -h.y,  h.z),
        Vector3D(-h.x,  h.y, -h.z),
        Vector3D(-h.x,  h.y,  h.z),
        Vector3D( h.x, -h.y, -h.z),
        Vector3D( h.x, -h.y,  h.z),
        Vector3D( h.x,  h.y, -h.z),
        Vector3D( h.x,  h.y,  h.z)
    };

    // 4. Test pour chaque sommet (Algorithme Chap 11)
    for (int i = 0; i < 8; i++)
    {
        // A. Passage Espace Monde : on applique la matrice de transformation
        Vector3D Q = transform * localSommets[i];

        // B. Distance signée t [Utilisation de la méthode helper de Plan]
        // Cela remplace le calcul manuel (n . (Q - P))
        float t = plan.getDistanceSignee(Q);

        // C. Test d'intersection (t <= 0)
        // Si t est négatif ou nul, le sommet est "derrière" la face avant du plan
        if (t <= 0.0f)
        {
            Contact nouveauContact;

            // Identification : On lie le contact au CorpsRigide interne de la boîte
            nouveauContact.c1 = &boite->body;
            nouveauContact.c2 = nullptr; // Plan statique (pas de corps rigide)

            // Propriétés physiques (Défauts, à ajuster selon tes matériaux)
            nouveauContact.restitution = 0.5f; 
            nouveauContact.friction = 0.5f;

            // Données de collision (Narrow Phase)
            nouveauContact.contactNormal = plan.normale;
            
            // Le point de contact est la projection du sommet sur le plan
            // R = Q - t * n (puisque t est la distance signée)
            nouveauContact.contactPoint = Q - (plan.normale * t);
            
            // La pénétration est la valeur absolue de t
            nouveauContact.penetration = -t;

            // Utilisation de la méthode add de CollisionData qui gère la liste
            data->add(nouveauContact);
        }
    }
}