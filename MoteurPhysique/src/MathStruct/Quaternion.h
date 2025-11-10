#pragma once
#include "3DVector.h"

class Quaternion {
public:
	float w, x, y, z;

	// --- Constructeurs ---
	Quaternion();                                   // zero
	Quaternion(float W, float X, float Y, float Z);

	// --- Création / Conversion ---
	static Quaternion Identity();
	static Quaternion FromAxisAngle(const Vector3D& axis, float angleRad);
	static Quaternion FromEulerXYZ(float rx, float ry, float rz);

	// --- Opérations de base ---
	float length() const;
	void  normalize();
	Quaternion normalized() const;
	Quaternion conjugate() const;
	Quaternion inverse() const;

	// --- Opérateurs ---
	Quaternion operator*(const Quaternion& b) const;  // composition
	Vector3D   rotate(const Vector3D& v) const;       // rotation d'un vecteur

	// --- Interpolation ---
	static Quaternion Slerp(const Quaternion& a, const Quaternion& b, float t);
};
