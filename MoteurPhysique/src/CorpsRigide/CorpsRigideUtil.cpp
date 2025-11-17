#pragma once
#include "CorpsRigide.h"

// halfExtents = (hx, hy, hz) = demi-tailles de la boîte
inline void SetupBoxRigidBody(CorpsRigide& body, float mass, const Vector3D& halfExtents)
{
	// Masse
	float invMass = (mass > 0.f) ? 1.f / mass : 0.f;
	body.setInverseMasse(invMass);

	// Inertie d'un pavé axis-aligned centré :
	// Ixx = 1/12 m (hy^2 + hz^2), etc.
	float hx = halfExtents.x;
	float hy = halfExtents.y;
	float hz = halfExtents.z;

	float Ixx = (mass / 12.f) * (hy*hy + hz*hz);
	float Iyy = (mass / 12.f) * (hx*hx + hz*hz);
	float Izz = (mass / 12.f) * (hx*hx + hy*hy);

	// Matrice d'inertie inverse dans l'espace corps
	float invIxx = (Ixx > 0.f) ? 1.f / Ixx : 0.f;
	float invIyy = (Iyy > 0.f) ? 1.f / Iyy : 0.f;
	float invIzz = (Izz > 0.f) ? 1.f / Izz : 0.f;

	Matrix3 invIbody(
		invIxx, 0.f,   0.f,
		0.f,   invIyy, 0.f,
		0.f,   0.f,   invIzz
	);

	body.setInverseInertiaTensorBody(invIbody);
}
