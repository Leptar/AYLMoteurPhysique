#pragma once
#include <array>

class Matrix4 {
public:
	std::array<float,16> m;

	Matrix4();
	explicit Matrix4(float diag);
	Matrix4(float m00,float m01,float m02,float m03,
			float m10,float m11,float m12,float m13,
			float m20,float m21,float m22,float m23,
			float m30,float m31,float m32,float m33);


	Matrix4 operator*(const Matrix4& B) const;
	Matrix4 operator-(const Matrix4 & B) const;
	Matrix4 operator+(const Matrix4 & B) const;
	Matrix4 operator/(const float & scalar) const;

	Matrix4 transpose() const;
	float   determinant() const;
	Matrix4 Identity() const {return Matrix4(1.f);}
	Matrix4 inverse() const;

};
