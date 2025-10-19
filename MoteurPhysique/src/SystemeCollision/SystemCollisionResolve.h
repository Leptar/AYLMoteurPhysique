#pragma once
#include "SystemCollisionDetection.h"

class SystemCollisionResolve
{
public:
    static void resolve(const Collision& collision);
    static void resolveConstraint(const Collision& collision);

private:
    static void seperate(const Collision& collision);
};
