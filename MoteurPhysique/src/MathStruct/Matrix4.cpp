#include "Matrix4.h"

#include "Matrix3.h"


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
	   m[0], m[1], m[2],  m[3],  // Col 0 devient Row 0
	   m[4], m[5], m[6],  m[7],  // Col 1 devient Row 1
	   m[8], m[9], m[10], m[11], // Col 2 devient Row 2
	   m[12], m[13], m[14], m[15] // Col 3 devient Row 3
	);
}

float Matrix4::determinant() const {
	float m00 = m[0], m01 = m[4], m02 = m[8],  m03 = m[12];
	float m10 = m[1], m11 = m[5], m12 = m[9],  m13 = m[13];
	float m20 = m[2], m21 = m[6], m22 = m[10], m23 = m[14];
	float m30 = m[3], m31 = m[7], m32 = m[11], m33 = m[15];

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

	if (std::abs(det) < 1e-6f) {
		return Identity();
	}

    float m00 = m[0], m01 = m[4], m02 = m[8],  m03 = m[12];
    float m10 = m[1], m11 = m[5], m12 = m[9],  m13 = m[13];
    float m20 = m[2], m21 = m[6], m22 = m[10], m23 = m[14];
    float m30 = m[3], m31 = m[7], m32 = m[11], m33 = m[15];

    float c00 = m11*(m22*m33 - m32*m23) - m12*(m21*m33 - m31*m23) + m13*(m21*m32 - m31*m22);
    float c01 = -(m10*(m22*m33 - m32*m23) - m12*(m20*m33 - m30*m23) + m13*(m20*m32 - m30*m22));
    float c02 = m10*(m21*m33 - m31*m23) - m11*(m20*m33 - m30*m23) + m13*(m20*m31 - m30*m21);
    float c03 = -(m10*(m21*m32 - m31*m22) - m11*(m20*m32 - m30*m22) + m12*(m20*m31 - m30*m21));

    float c10 = -(m01*(m22*m33 - m32*m23) - m02*(m21*m33 - m31*m23) + m03*(m21*m32 - m31*m22));
    float c11 = m00*(m22*m33 - m32*m23) - m02*(m20*m33 - m30*m23) + m03*(m20*m32 - m30*m22);
    float c12 = -(m00*(m21*m33 - m31*m23) - m01*(m20*m33 - m30*m23) + m03*(m20*m31 - m30*m21));
    float c13 = m00*(m21*m32 - m31*m22) - m01*(m20*m32 - m30*m22) + m02*(m20*m31 - m30*m21);

    float c20 = m01*(m12*m33 - m32*m13) - m02*(m11*m33 - m31*m13) + m03*(m11*m32 - m31*m12);
    float c21 = -(m00*(m12*m33 - m32*m13) - m02*(m10*m33 - m30*m13) + m03*(m10*m32 - m30*m12));
    float c22 = m00*(m11*m33 - m31*m13) - m01*(m10*m33 - m30*m13) + m03*(m10*m31 - m30*m11);
    float c23 = -(m00*(m11*m32 - m31*m12) - m01*(m10*m32 - m30*m12) + m02*(m10*m31 - m30*m11));

    float c30 = -(m01*(m12*m23 - m22*m13) - m02*(m11*m23 - m21*m13) + m03*(m11*m22 - m21*m12));
    float c31 = m00*(m12*m23 - m22*m13) - m02*(m10*m23 - m20*m13) + m03*(m10*m22 - m20*m12);
    float c32 = -(m00*(m11*m23 - m21*m13) - m01*(m10*m23 - m20*m13) + m03*(m10*m21 - m20*m11));
    float c33 = m00*(m11*m22 - m21*m12) - m01*(m10*m22 - m20*m12) + m02*(m10*m21 - m20*m11);

    return Matrix4(
        c00, c10, c20, c30,
        c01, c11, c21, c31,
        c02, c12, c22, c32,
        c03, c13, c23, c33
    ) / det;
}

void Matrix4::SetRotation(Matrix3& m_rotation) {
	Matrix3 m_transfo_linear(m[0], m[1], m[2],
							m[4], m[5], m[6],
							m[8], m[9], m[10]);
	m_transfo_linear = m_transfo_linear * m_rotation;

	m[0] = m_transfo_linear.m[0];
	m[1] = m_transfo_linear.m[1];
	m[2] = m_transfo_linear.m[2];
	m[4] = m_transfo_linear.m[4];
	m[5] = m_transfo_linear.m[5];
	m[6] = m_transfo_linear.m[6];
	m[8] = m_transfo_linear.m[8];
	m[9] = m_transfo_linear.m[9];
	m[10] = m_transfo_linear.m[10];

}

Matrix4 Matrix4::SetPosition(Vector3D & m_position)
{
	m[3] = m_position.x;
	m[7] = m_position.y;
	m[11] = m_position.z;
}


