#include "Quaternion.h"

#include <Matrix3.h>
#include <algorithm>
#include <cmath>

Quaternion::Quaternion() : w(0.f), x(0.f), y(0.f), z(0.f) {}
Quaternion::Quaternion(float W, float X, float Y, float Z) : w(W), x(X), y(Y), z(Z) {}

Quaternion Quaternion::Identity() { return Quaternion(1.f, 0.f, 0.f, 0.f); }

Quaternion Quaternion::FromAxisAngle(const Vector3D& axis, float angleRad) {

    Vector3D n = axis;
    n.normalize();

    float half = 0.5f * angleRad;
    float s = std::sin(half);
    return Quaternion(std::cos(half), n.x*s, n.y*s, n.z*s);
}


Quaternion Quaternion::FromEulerXYZ(float rx, float ry, float rz) {
    // rotations locales X -> Y -> Z
    Quaternion qx = FromAxisAngle(Vector3D(1,0,0), rx);
    Quaternion qy = FromAxisAngle(Vector3D(0,1,0), ry);
    Quaternion qz = FromAxisAngle(Vector3D(0,0,1), rz);
    return qz * (qy * qx);
}

float Quaternion::length() const {
    return std::sqrt(w*w + x*x + y*y + z*z);
}

void Quaternion::normalize() {
    float len = length();
    if (len == 0.f) { w = 1.f; x = y = z = 0.f; return; }
    float inv = 1.f / len;
    w *= inv; x *= inv; y *= inv; z *= inv;
}

Quaternion Quaternion::normalized() const {
    Quaternion q = *this;
    q.normalize();
    return q;
}

Quaternion Quaternion::conjugate() const {
    return Quaternion(w, -x, -y, -z);
}

Quaternion Quaternion::inverse() const {
    float L2 = w*w + x*x + y*y + z*z;
    if (L2 == 0.f) return *this;
    Quaternion c = conjugate();
    return Quaternion(c.w/L2, c.x/L2, c.y/L2, c.z/L2);
}

Quaternion Quaternion::scalar(float f) {
    return Quaternion(f*w, f*x, f*y, f*z);
}

Quaternion Quaternion::operator*(const Quaternion& b) const {
    const Quaternion& a = *this;
    return Quaternion(
        a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
        a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
        a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
        a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w
    );
}

Vector3D Quaternion::rotate(const Vector3D& v) const {
    // q * v * q^{-1}
    Quaternion qv(0, v.x, v.y, v.z);
    Quaternion r = (*this) * qv * this->inverse();
    return Vector3D(r.x, r.y, r.z);
}

Matrix3 Quaternion::toMatrix3() const {
	float xx = x * x;
	float yy = y * y;
	float zz = z * z;
	float xy = x * y;
	float xz = x * z;
	float yz = y * z;
	float wx = w * x;
	float wy = w * y;
	float wz = w * z;

	return Matrix3(
		1.f - 2.f * (yy + zz), 2.f * (xy - wz),       2.f * (xz + wy),
		2.f * (xy + wz),       1.f - 2.f * (xx + zz), 2.f * (yz - wx),
		2.f * (xz - wy),       2.f * (yz + wx),       1.f - 2.f * (xx + yy)
	);
}

Quaternion Quaternion::Slerp(const Quaternion& a, const Quaternion& b, float t) {
	if (t <= 0.f) return a;
	if (t >= 1.f) return b;

	float dot = a.w*b.w + a.x*b.x + a.y*b.y + a.z*b.z;
	Quaternion bb = b;
	if (dot < 0.f) { dot = -dot; bb = Quaternion(-b.w, -b.x, -b.y, -b.z); }

	const float EPS = 1e-6f;

	if (1.f - dot < EPS) {
		// nlerp
		Quaternion r(
			a.w + (bb.w - a.w)*t,
			a.x + (bb.x - a.x)*t,
			a.y + (bb.y - a.y)*t,
			a.z + (bb.z - a.z)*t
		);
		r.normalize();
		return r;
	}

	dot = std::clamp(dot, -1.f, 1.f);
	float theta  = std::acos(dot);
	float s0     = std::sin((1.f - t) * theta);
	float s1     = std::sin(t * theta);
	float invSin = 1.f / std::sin(theta);

	Quaternion r(
		(a.w*s0 + bb.w*s1) * invSin,
		(a.x*s0 + bb.x*s1) * invSin,
		(a.y*s0 + bb.y*s1) * invSin,
		(a.z*s0 + bb.z*s1) * invSin
	);
	return r;
}

float Quaternion::dot(const Quaternion& b) const {
	return w * b.w + x * b.x + y * b.y + z * b.z;
}

Quaternion Quaternion::operator+(const Quaternion & b) const{
	return Quaternion(w + b.w, x + b.x, y + b.y, z + b.z);
}
