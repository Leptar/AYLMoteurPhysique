#pragma once

#include <memory>

#include "Box.h"
#include "../CorpsRigide/CorpsRigide.h"
#include "../MathStruct/Matrix4.h"
#include "ofColor.h"

struct RigidBodyBox
{
        CorpsRigide body;
        Vector3D halfExtents{1.f, 1.f, 1.f};
        float mass = 1.f;
        ofColor color = ofColor::white;
        float boundingRadius = 1.f;
        bool reachedGoal = false;
        bool outOfBounds = false;
        std::unique_ptr<Box> primitive;

        void syncPrimitive()
        {
                if (!primitive)
                {
                        primitive = std::make_unique<Box>(halfExtents);
                }
                primitive->HalfExtent = halfExtents;
                primitive->corpsRigide = &body;
                primitive->offset = Matrix4::Identity();
        }
};

