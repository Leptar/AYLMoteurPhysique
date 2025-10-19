#pragma once
#include "SystemCollisionDetection.h"

class SystemCollisionResolve
{
public:
    /// Resolve a standard contact by applying an impulse and positional correction.
    static void resolve(const Collision& collision);
    /// Enforce distance constraints such as springs by projecting particles apart.
    static void resolveConstraint(const Collision& collision);

private:
    /// Separate two overlapping bodies using their contact normal and inverse masses.
    static void seperate(const Collision& collision);
};
