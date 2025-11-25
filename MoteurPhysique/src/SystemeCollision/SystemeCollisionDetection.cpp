#include "SystemCollisionDetection.h"

// Inclusions nécessaires pour que le compilateur connaisse la structure interne des classes
#include "../World/RigidBodyBox.h" 
#include "../MathStruct/Matrix4.h" 
#include "../MathStruct/3DVector.h"
#include <cmath> // Pour std::abs si besoin

void SystemCollisionDetection::genererContactsBoitePlan(
    RigidBodyBox* boite, 
    const Plan& plan, 
    CollisionData* data)
{
    // Sécurité : on vérifie que les pointeurs sont valides
    if (!boite || !data) return;

    // 1. Récupération des dimensions (Demi-mesures)
    // Adaptation à votre structure RigidBodyBox qui utilise 'halfExtents'
    Vector3D h = boite->halfExtents; 

    // 2. Récupération de la Matrice de transformation (Locale -> Monde)
    // La matrice appartient au CorpsRigide 'body' contenu dans la boîte
    // Note: Assurez-vous que CorpsRigide a une méthode getTransform() ou un membre public transformMatrix
    Matrix4 transform = boite->body.getTransform(); 

    // 3. Définition des 8 sommets de la boîte dans l'espace LOCAL
    // (Les coins relatifs au centre de la boîte (0,0,0))
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

    // 4. Algorithme Boîte-Plan (Chapitre 11, Page 2)
    for (int i = 0; i < 8; i++)
    {
        // A. Transformation du sommet de l'espace Local vers l'espace Monde
        Vector3D Q = transform * localSommets[i];

        // B. Calcul de la distance signée t [Chap 11, Page 1]
        // Formule : t = n . (Q - P)
        float t = plan.getDistanceSignee(Q);

        // C. Vérification de l'intersection (Phase Restreinte)
        // Si t <= 0, le point est "derrière" ou "sur" le plan, donc il y a contact.
        if (t <= 0.0f)
        {
            Contact nouveauContact;

            // Identification des corps impliqués
            // Attention : c1 est un CorpsRigide*, or boite est un RigidBodyBox*
            // On passe donc l'adresse du CorpsRigide contenu dans la structure
            nouveauContact.c1 = &boite->body;
            nouveauContact.c2 = nullptr; // Le plan est statique

            // Propriétés physiques (valeurs par défaut, à ajuster selon les matériaux)
            nouveauContact.restitution = 0.5f; 
            nouveauContact.friction = 0.5f;

            // Normale : C'est la normale du plan
            nouveauContact.contactNormal = plan.normale;

            // Point d'impact R : R = Q - t * n
            // (Projection du sommet sur le plan le long de la normale)
            nouveauContact.contactPoint = Q - (plan.normale * t);

            // Pénétration : Valeur absolue de t (car t est négatif ou nul)
            nouveauContact.penetration = -t;

            // Ajout du contact généré
            data->add(nouveauContact);
        }
    }
}