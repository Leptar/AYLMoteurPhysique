#pragma once
#include "SystemCollisionDetection.h"

class SystemCollisionResolve
{
public:
    static void resolve(Collision collision);
    
private:
    static void seperate(Collision collision);
};
