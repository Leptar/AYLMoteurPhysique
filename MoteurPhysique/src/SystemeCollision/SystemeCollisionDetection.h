#pragma once

#include <vector>
#include "CollisionData.h"         
#include "../World/RigidBodyBox.h" 
#include "3DVector.h"

// Définition de la Primitive Plan nécessaire pour l'algorithme
// Basé sur le document 11, page 1 : défini par normale (n) et un point (P)
class Plan {
public:
    Vector3D normale;      // n
    Vector3D pointSurPlan; // P

    Plan(Vector3D n, Vector3D p) : normale(n), pointSurPlan(p) {
        normale.normalize();
    }

    // Calcul de la distance signée t = n . (Q - P) [cite: 9, 16]
    float getDistanceSignee(const Vector3D& point) const {
        return normale.dot(point - pointSurPlan);
    }
};

class SystemeCollisionDetection
{
public:
    /**
     * Phase Restreinte : Génération de contacts Boîte-Plan
     * Algorithme : "Pour chaque sommet de la boîte, tester s'il est en-dessous du plan" [cite: 24]
     * @param boite : Le corps rigide dynamique (c1)
     * @param plan : La primitive statique
     * @param data : La structure de données à remplir
     */
    static void genererContactsBoitePlan(
        RigidBodyBox* boite, 
        const Plan& plan, 
        CollisionData* data
    );
};