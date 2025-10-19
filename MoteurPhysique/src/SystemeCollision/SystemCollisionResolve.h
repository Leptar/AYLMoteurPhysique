#pragma once
#include "SystemCollisionDetection.h"

class SystemCollisionResolve
{
public:
    /// Résout un contact standard en appliquant une impulsion et une correction de position.
    static void resolve(const Collision& collision);
    /// Fait respecter les contraintes de distance comme les ressorts en projetant les particules.
    static void resolveConstraint(const Collision& collision);

private:
    /// Sépare deux corps qui se chevauchent en utilisant leur normale de contact et leurs masses inverses.
    static void seperate(const Collision& collision);
};
