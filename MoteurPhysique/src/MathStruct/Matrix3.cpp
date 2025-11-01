#include "Matrix3.h"
#include <cmath>

Matrix3::Matrix3() { setIdentity(); }

Matrix3::Matrix3(float diag) {
    setIdentity();
    m[0] = m[4] = m[8] = diag;
}

Matrix3::Matrix3(float m00,float m01,float m02,
                 float m10,float m11,float m12,
                 float m20,float m21,float m22) {
    m = { m00,m10,m20,  m01,m11,m21,  m02,m12,m22 };
}

Matrix3 Matrix3::Identity() { return Matrix3(1.f); }

void Matrix3::setIdentity() { m = {1,0,0, 0,1,0, 0,0,1}; }

float& Matrix3::operator()(int r,int c)       { return m[c*3 + r]; }
float  Matrix3::operator()(int r,int c) const { return m[c*3 + r]; }

Matrix3 Matrix3::FromColumns(const Vector3D& c0, const Vector3D& c1, const Vector3D& c2) {
    Matrix3 R;
    R(0,0)=c0.x; R(1,0)=c0.y; R(2,0)=c0.z;
    R(0,1)=c1.x; R(1,1)=c1.y; R(2,1)=c1.z;
    R(0,2)=c2.x; R(1,2)=c2.y; R(2,2)=c2.z;
    return R;
}

Matrix3 Matrix3::operator*(const Matrix3& B) const {
    Matrix3 C; C.m = {0,0,0,0,0,0,0,0,0};
    for (int c=0;c<3;++c)
        for (int r=0;r<3;++r)
            for (int k=0;k<3;++k)
                C(r,c) += (*this)(r,k) * B(k,c);
    return C;
}

Vector3D Matrix3::operator*(const Vector3D& v) const {
    return Vector3D(
        (*this)(0,0)*v.x + (*this)(0,1)*v.y + (*this)(0,2)*v.z,
        (*this)(1,0)*v.x + (*this)(1,1)*v.y + (*this)(1,2)*v.z,
        (*this)(2,0)*v.x + (*this)(2,1)*v.y + (*this)(2,2)*v.z
    );
}

Matrix3 Matrix3::operator-(const Matrix3& B) const {
	Matrix3 C;
	for (int i=0;i<9;++i)
		C.m[i] = m[i] - B.m[i];
	return C;
}

Matrix3 Matrix3::operator+(const Matrix3& B) const {
	Matrix3 C;
	for (int i=0;i<9;++i)
		C.m[i] = m[i] + B.m[i];
	return C;
}

Matrix3 Matrix3::operator/(const float& scalar) const {
	Matrix3 C;
	for (int i=0;i<9;++i)
		C.m[i] = m[i] / scalar;
	return C;
}

Matrix3 Matrix3::transpose() const {
    return Matrix3(
        (*this)(0,0), (*this)(1,0), (*this)(2,0),
        (*this)(0,1), (*this)(1,1), (*this)(2,1),
        (*this)(0,2), (*this)(1,2), (*this)(2,2)
    );
}

float Matrix3::determinant() const {
    const float a=(*this)(0,0), b=(*this)(0,1), c=(*this)(0,2);
    const float d=(*this)(1,0), e=(*this)(1,1), f=(*this)(1,2);
    const float g=(*this)(2,0), h=(*this)(2,1), i=(*this)(2,2);
    return a*(e*i - f*h) - b*(d*i - f*g) + c*(d*h - e*g);
}

Matrix3 Matrix3::inverse() const {
    const float a=(*this)(0,0), b=(*this)(0,1), c=(*this)(0,2);
    const float d=(*this)(1,0), e=(*this)(1,1), f=(*this)(1,2);
    const float g=(*this)(2,0), h=(*this)(2,1), i=(*this)(2,2);
    float det = determinant();
    if (std::fabs(det) < 1e-8f) return Matrix3(0.f);
    Matrix3 inv(
         (e*i - f*h), -(b*i - c*h),  (b*f - c*e),
        -(d*i - f*g),  (a*i - c*g), -(a*f - c*d),
         (d*h - e*g), -(a*h - b*g),  (a*e - b*d)
    );
    inv = inv/det;
    return inv;
}

// Rotations
Matrix3 Matrix3::RotX(float a) {
    float c=std::cos(a), s=std::sin(a);
    return Matrix3(1,0,0, 0,c,-s, 0,s,c);
}
Matrix3 Matrix3::RotY(float a) {
    float c=std::cos(a), s=std::sin(a);
    return Matrix3(c,0,s, 0,1,0, -s,0,c);
}
Matrix3 Matrix3::RotZ(float a) {
    float c=std::cos(a), s=std::sin(a);
    return Matrix3(c,-s,0, s,c,0, 0,0,1);
}
Matrix3 Matrix3::AxisAngle(const Vector3D& axis, float angle) {
    float x=axis.x, y=axis.y, z=axis.z;
    float len = std::sqrt(x*x+y*y+z*z);
    if (len==0) return Identity();
    x/=len; y/=len; z/=len;
    float c=std::cos(angle), s=std::sin(angle), t=1.f-c;
    return Matrix3(
        t*x*x + c,   t*x*y - s*z, t*x*z + s*y,
        t*y*x + s*z, t*y*y + c,   t*y*z - s*x,
        t*z*x - s*y, t*z*y + s*x, t*z*z + c
    );
}
