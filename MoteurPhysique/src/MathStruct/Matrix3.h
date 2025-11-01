#pragma once
#include <array>
#include "3DVector.h"

class Matrix3 {
public:
	// column-major: m[c*3 + r]
	std::array<float,9> m;

	Matrix3();
	explicit Matrix3(float diag);
	Matrix3(float m00,float m01,float m02,
			float m10,float m11,float m12,
			float m20,float m21,float m22);

	static Matrix3 Identity();
	void setIdentity();

	float& operator()(int r,int c);
	float  operator()(int r,int c) const;

	static Matrix3 FromColumns(const Vector3D& c0, const Vector3D& c1, const Vector3D& c2);

	Matrix3 operator*(const Matrix3& B) const;
	Vector3D operator*(const Vector3D& v) const;
	Matrix3 operator-(const Matrix3 & B) const;
	Matrix3 operator+(const Matrix3 & B) const;
	Matrix3 operator/(const float & scalar) const;

	Matrix3 transpose() const;
	float   determinant() const;
	Matrix3 inverse() const;

	static Matrix3 RotX(float a);
	static Matrix3 RotY(float a);
	static Matrix3 RotZ(float a);
	static Matrix3 AxisAngle(const Vector3D& axis, float angle);
};
