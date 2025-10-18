#include "SystemCollisionResolve.h"

void SystemCollisionResolve::resolve(Collision collision)
{
    if (collision.penetration < 0.f) {seperate(collision);}
    
}

void SystemCollisionResolve::seperate(Collision collision)
{
    
}
