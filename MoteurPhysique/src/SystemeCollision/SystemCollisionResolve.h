#pragma once
#include "SystemCollisionDetection.h"

class SystemCollisionResolve
{
public:
    static void resolve(Collision collision);
    static void resolveConstraint(const Collision& collision);

private:
    static void seperate(Collision collision);
};
