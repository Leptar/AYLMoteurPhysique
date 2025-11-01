#include "Matrix4.h"

Matrix4::Matrix4() {
	m = {1,0,0,0,
		 0,1,0,0,
		 0,0,1,0,
		 0,0,0,1};
}

Matrix4::Matrix4(float diag) {
	m = {diag,0,0,0,
		 0,diag,0,0,
		 0,0,diag,0,
		 0,0,0,diag};
}

Matrix4::Matrix4(float m00, float m01, float m02, float m03, float m10, float m11, float m12, float m13, float m20, float m21, float m22, float m23, float m30, float m31, float m32, float m33) {
	m = {m00, m10, m20, m30,
		 m01, m11, m21, m31,
		 m02, m12, m22, m32,
		 m03, m13, m23, m33};
}

Matrix4 Matrix4::operator/(const float & scalar) const {
	Matrix4 C;
	for (int i=0;i<16;++i)
		C.m[i] = m[i] / scalar;
	return C;
}

Matrix4 Matrix4::operator+(const Matrix4 & B) const {
	Matrix4 C;
	for (int i=0;i<16;++i)
		C.m[i] = m[i] + B.m[i];
	return C;
}
Matrix4 Matrix4::operator-(const Matrix4 & B) const {
	Matrix4 C;
	for (int i=0;i<16;++i)
		C.m[i] = m[i] - B.m[i];
	return C;
}

Matrix4 Matrix4::operator*(const Matrix4 & B) const {
	Matrix4 C;
	for (int c=0;c<4;++c)
		for (int r=0;r<4;++r)
			for (int k=0;k<4;++k)
				C.m[c*4 + r] += m[k*4 + r] * B.m[c*4 + k];
	return C;
}

Matrix4 Matrix4::transpose() const {
	return Matrix4(
		m[0], m[4], m[8],  m[12],
		m[1], m[5], m[9],  m[13],
		m[2], m[6], m[10], m[14],
		m[3], m[7], m[11], m[15]
	);
}

float Matrix4::determinant() const {
    float m00 = m[0],  m01 = m[4],  m02 = m[8],  m03 = m[12];
    float m10 = m[1],  m11 = m[5],  m12 = m[9],  m13 = m[13];
    float m20 = m[2],  m21 = m[6],  m22 = m[10], m23 = m[14];
    float m30 = m[3],  m31 = m[7],  m32 = m[11], m33 = m[15];

    auto det3 = [](float a, float b, float c,
                   float d, float e, float f,
                   float g, float h, float i) -> float {
        return a * (e * i - f * h)
             - b * (d * i - f * g)
             + c * (d * h - e * g);
    };

    float det =
        m00 * det3(m11, m12, m13,  m21, m22, m23,  m31, m32, m33)
      - m01 * det3(m10, m12, m13,  m20, m22, m23,  m30, m32, m33)
      + m02 * det3(m10, m11, m13,  m20, m21, m23,  m30, m31, m33)
      - m03 * det3(m10, m11, m12,  m20, m21, m22,  m30, m31, m32);

    return det;
}

Matrix4 Matrix4::inverse() const {
	float det = determinant();
	if (det == 0.0f) {
		return Matrix4::Identity();
	}
	Matrix4 inv= transpose();
	inv = inv / det;
	return inv;
}


