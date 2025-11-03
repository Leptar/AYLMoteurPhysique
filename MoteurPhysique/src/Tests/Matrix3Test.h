#pragma once
#include "Matrix3.h"

class Matrix3Test {

public:
	Matrix3Test() {
		AMatrix = Matrix3(1.f, 2.f, 3.f, 4.f, 5.f, 6.f, 7.f, 8.f, 9.f);
		BMatrix = Matrix3(1.f, 4.f, 7.f, 2.f, 5.f, 8.f, 3.f, 6.f, 9.f);
		CMatrix = Matrix3(1.f,5.f,9.f,7.f,3.f,6.f,8.f,4.f,2.f);
	}

	Matrix3 AMatrix;
	Matrix3 BMatrix;
	Matrix3 Result;
	Matrix3 CMatrix;

	// Operateur
	void TestAddition();
	void TestSubtraction();
	void TestMultiplication();
	void TestDivision();

	// Methods
	void TestTranspose();
	void TestInverse();
	void TestDeterminant();

	// Rotation
	void TestRotationZero();
	void TestRotation_90Degrees();
	void TestRotation_Transformation();

	void Test_AxisAngle_EdgeCases();
	void test_axis_angle_equivalence();
	void Test_AxisAngle_Normalization();
	void Test_Property_Orthogonal();

	void RunAllTests();

};
